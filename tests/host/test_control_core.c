#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "control_manager.h"

static int failures;
#define CHECK(expr) do { if (!(expr)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); ++failures; } } while (0)

typedef struct {
    bool motor_sleep;
    bool servo_disabled;
    int motor_a;
    int motor_b;
    int servo_angle;
    bool supply_on;
    bool power_good;
    bool coil_on;
} fake_actuators_t;

static esp_err_t fake_sleep(void *ctx, bool sleep) { ((fake_actuators_t *)ctx)->motor_sleep = sleep; return ESP_OK; }
static esp_err_t fake_motors(void *ctx, int16_t a, int16_t b) { fake_actuators_t *f = ctx; f->motor_a = a; f->motor_b = b; return ESP_OK; }
static esp_err_t fake_servo(void *ctx, int16_t angle) { fake_actuators_t *f = ctx; f->servo_angle = angle; f->servo_disabled = false; return ESP_OK; }
static esp_err_t fake_disable_servo(void *ctx) { ((fake_actuators_t *)ctx)->servo_disabled = true; return ESP_OK; }
static esp_err_t fake_coil(void *ctx, bool enabled)
{
    fake_actuators_t *fake = ctx;
    CHECK(!enabled || (fake->supply_on && fake->power_good));
    fake->coil_on = enabled;
    return ESP_OK;
}

static esp_err_t fake_supply(void *ctx, bool enabled)
{
    fake_actuators_t *fake = ctx;
    CHECK(!fake->coil_on);
    fake->supply_on = enabled;
    return ESP_OK;
}

static bool fake_power_good(void *ctx)
{
    return ((fake_actuators_t *)ctx)->power_good;
}

static control_manager_t make_manager(fake_actuators_t *fake, robot_control_config_t config)
{
    control_manager_t manager;
    const robot_actuator_ops_t ops = {
        .set_motor_sleep = fake_sleep, .set_motors = fake_motors,
        .set_servo = fake_servo, .disable_servo = fake_disable_servo,
        .set_coil_enabled = fake_coil,
        .set_sol_supply_enabled = fake_supply,
        .read_sol_power_good = fake_power_good,
    };
    memset(fake, 0, sizeof(*fake));
    CHECK(control_manager_init(&manager, &config, &ops, fake) == ESP_OK);
    CHECK(fake->motor_sleep && fake->motor_a == 0 && fake->motor_b == 0);
    CHECK(fake->servo_disabled);
    CHECK(control_manager_boot_complete(&manager) == ESP_OK);
    return manager;
}

static robot_command_t command(robot_command_type_t type, uint32_t seq, int64_t time_us)
{
    robot_command_t value;
    memset(&value, 0, sizeof(value));
    value.type = type; value.sequence = seq; value.received_at_us = time_us;
    value.source = ROBOT_COMMAND_SOURCE_WIFI;
    return value;
}

static void arm(control_manager_t *manager, uint32_t seq, int64_t now_us)
{
    robot_command_t value = command(ROBOT_CMD_ARM, seq, now_us);
    CHECK(control_manager_submit(manager, &value, now_us) == ROBOT_COMMAND_ACCEPTED);
    CHECK(manager->state_machine.state == ROBOT_STATE_READY);
}

static void test_arm_gate_and_mix(void)
{
    fake_actuators_t fake;
    control_manager_t manager = make_manager(&fake, control_manager_default_config());
    robot_command_t drive = command(ROBOT_CMD_DRIVE, 1, 1000);
    drive.value.drive.throttle = 25; drive.value.drive.steering = 10;
    CHECK(control_manager_submit(&manager, &drive, 1000) == ROBOT_COMMAND_INVALID_STATE);
    CHECK(fake.motor_a == 0 && fake.motor_b == 0);
    arm(&manager, 2, 2000);
    CHECK(!fake.motor_sleep);
    int16_t a = 0, b = 0;
    control_manager_mix_drive(80, 40, &a, &b);
    CHECK(a == 100 && b == 40);
    control_manager_mix_drive(-80, -40, &a, &b);
    CHECK(a == -100 && b == -40);
}

static void test_ramp_and_reverse_deadtime(void)
{
    fake_actuators_t fake;
    robot_control_config_t config = control_manager_default_config();
    control_manager_t manager = make_manager(&fake, config);
    arm(&manager, 1, 1000000);
    robot_command_t motors = command(ROBOT_CMD_MOTOR_DIRECT, 2, 1000000);
    motors.value.motors.motor_a = 50; motors.value.motors.motor_b = -50;
    CHECK(control_manager_submit(&manager, &motors, 1000000) == ROBOT_COMMAND_ACCEPTED);
    CHECK(control_manager_tick(&manager, 1010000) == ESP_OK);
    CHECK(fake.motor_a == 5 && fake.motor_b == -5);

    config.motor_ramp_percent_per_10ms = 100; config.reverse_deadtime_ms = 30;
    manager = make_manager(&fake, config); arm(&manager, 1, 1000000);
    motors = command(ROBOT_CMD_MOTOR_DIRECT, 2, 1000000);
    motors.value.motors.motor_a = 100;
    CHECK(control_manager_submit(&manager, &motors, 1000000) == ROBOT_COMMAND_ACCEPTED);
    CHECK(control_manager_tick(&manager, 1010000) == ESP_OK && fake.motor_a == 100);
    motors = command(ROBOT_CMD_MOTOR_DIRECT, 3, 1020000);
    motors.value.motors.motor_a = -100;
    CHECK(control_manager_submit(&manager, &motors, 1020000) == ROBOT_COMMAND_ACCEPTED);
    CHECK(control_manager_tick(&manager, 1020000) == ESP_OK && fake.motor_a == 0);
    CHECK(control_manager_tick(&manager, 1040000) == ESP_OK && fake.motor_a == 0);
    CHECK(control_manager_tick(&manager, 1050000) == ESP_OK && fake.motor_a == 0);
    CHECK(control_manager_tick(&manager, 1060000) == ESP_OK && fake.motor_a == -100);
}

static void test_expiry_sequence_and_estop_priority(void)
{
    fake_actuators_t fake;
    control_manager_t manager = make_manager(&fake, control_manager_default_config());
    arm(&manager, 10, 1000000);
    robot_command_t drive = command(ROBOT_CMD_DRIVE, 11, 1000000);
    drive.value.drive.throttle = 20;
    const int64_t expired_at_us = drive.received_at_us +
                                  manager.config.command_max_age_ms * 1000LL + 1;
    CHECK(control_manager_submit(&manager, &drive, expired_at_us) == ROBOT_COMMAND_EXPIRED);
    drive.received_at_us = expired_at_us + 1000;
    CHECK(control_manager_submit(&manager, &drive, drive.received_at_us) == ROBOT_COMMAND_ACCEPTED);
    CHECK(control_manager_submit(&manager, &drive, drive.received_at_us) == ROBOT_COMMAND_OUT_OF_ORDER);
    robot_command_t estop = command(ROBOT_CMD_ESTOP, 1, 0);
    CHECK(control_manager_submit(&manager, &estop, 9000000) == ROBOT_COMMAND_ACCEPTED);
    CHECK(manager.state_machine.state == ROBOT_STATE_ESTOP);
    CHECK(fake.motor_sleep && fake.motor_a == 0 && fake.motor_b == 0);
}

static void test_watchdog_and_fault_clear(void)
{
    fake_actuators_t fake;
    robot_control_config_t config = control_manager_default_config();
    CHECK(config.command_timeout_ms == 3000);
    config.command_timeout_ms = 100;
    control_manager_t manager = make_manager(&fake, config);
    arm(&manager, 1, 1000000);
    robot_command_t drive = command(ROBOT_CMD_DRIVE, 2, 1000000);
    drive.value.drive.throttle = 20;
    CHECK(control_manager_submit(&manager, &drive, 1000000) == ROBOT_COMMAND_ACCEPTED);
    CHECK(control_manager_tick(&manager, 1099999) == ESP_OK);
    CHECK(manager.state_machine.state == ROBOT_STATE_RUNNING);
    CHECK(control_manager_tick(&manager, 1100001) == ESP_ERR_TIMEOUT);
    CHECK(manager.state_machine.state == ROBOT_STATE_SAFE);
    CHECK(!manager.state_machine.estop_latched);
    CHECK(manager.state_machine.faults == ROBOT_FAULT_NONE);
    CHECK(fake.motor_sleep && fake.motor_a == 0 && fake.motor_b == 0);
    robot_command_t rearm = command(ROBOT_CMD_ARM, 3, 1100001);
    CHECK(control_manager_submit(&manager, &rearm, 1100001) == ROBOT_COMMAND_ACCEPTED);

    manager = make_manager(&fake, control_manager_default_config());
    arm(&manager, 1, 2000000);
    control_manager_report_hardware_fault(&manager, ROBOT_FAULT_DRV8833);
    CHECK(manager.state_machine.state == ROBOT_STATE_FAULT);
    robot_command_t clear = command(ROBOT_CMD_CLEAR_FAULT, 2, 2000000);
    CHECK(control_manager_submit(&manager, &clear, 2000000) == ROBOT_COMMAND_INVALID_STATE);
    control_manager_clear_hardware_fault(&manager, ROBOT_FAULT_DRV8833);
    clear.sequence = 3;
    CHECK(control_manager_submit(&manager, &clear, 2000000) == ROBOT_COMMAND_ACCEPTED);
    CHECK(manager.state_machine.state == ROBOT_STATE_SAFE);
    CHECK(manager.state_machine.faults == ROBOT_FAULT_NONE);
}

static void test_servo_range_slew_and_disable(void)
{
    fake_actuators_t fake;
    robot_control_config_t config = control_manager_default_config();
    config.servo_slew_deg_per_second = 100;
    control_manager_t manager = make_manager(&fake, config);
    arm(&manager, 1, 1000000);

    robot_command_t servo = command(ROBOT_CMD_SERVO, 2, 1000000);
    servo.value.servo.angle_deg = 181;
    CHECK(control_manager_submit(&manager, &servo, 1000000) == ROBOT_COMMAND_INVALID_ARGUMENT);
    CHECK(fake.servo_disabled);

    servo.sequence = 3;
    servo.value.servo.angle_deg = 120;
    CHECK(control_manager_submit(&manager, &servo, 1000000) == ROBOT_COMMAND_ACCEPTED);
    CHECK(fake.servo_disabled);
    CHECK(control_manager_tick(&manager, 1010000) == ESP_OK);
    CHECK(!fake.servo_disabled && fake.servo_angle == 91);
    CHECK(control_manager_tick(&manager, 1110000) == ESP_OK);
    CHECK(fake.servo_angle == 101);

    robot_command_t disarm = command(ROBOT_CMD_DISARM, 4, 1110000);
    CHECK(control_manager_submit(&manager, &disarm, 1110000) == ROBOT_COMMAND_ACCEPTED);
    CHECK(fake.servo_disabled);
}

static robot_command_result_t submit_power(control_manager_t *manager,
                                            uint32_t sequence, int64_t now_us,
                                            bool asserted, bool diagnostic)
{
    robot_command_t request = command(ROBOT_CMD_POWER_CONTROL, sequence, now_us);
    request.source = diagnostic ? ROBOT_COMMAND_SOURCE_SERIAL : ROBOT_COMMAND_SOURCE_WIFI;
    request.value.power.asserted = asserted;
    request.value.power.diagnose_supply = diagnostic;
    return control_manager_submit(manager, &request, now_us);
}

static void test_solenoid_pulse_and_cooldown(void)
{
    fake_actuators_t fake;
    robot_control_config_t config = control_manager_default_config();
    control_manager_t manager = make_manager(&fake, config);
    CHECK(submit_power(&manager, 1, 1000, true, false) == ROBOT_COMMAND_INVALID_STATE);
    CHECK(!fake.supply_on && !fake.coil_on);

    robot_actuator_ops_t ops = manager.actuator_ops;
    ops.sol_pulse_ms = 50;
    ops.sol_cooldown_ms = 100;
    CHECK(control_manager_init(&manager, &config, &ops, &fake) == ESP_OK);
    CHECK(control_manager_boot_complete(&manager) == ESP_OK);
    CHECK(submit_power(&manager, 1, 1000, true, false) == ROBOT_COMMAND_ACCEPTED);
    CHECK(fake.supply_on && !fake.coil_on);
    CHECK(control_manager_tick(&manager, 10000) == ESP_OK);
    CHECK(!fake.coil_on);

    fake.power_good = true;
    CHECK(control_manager_tick(&manager, 20000) == ESP_OK);
    CHECK(fake.coil_on && manager.status.power_asserted && manager.status.sol_power_good);
    CHECK(submit_power(&manager, 2, 60000, true, false) == ROBOT_COMMAND_INVALID_STATE);
    CHECK(control_manager_tick(&manager, 70000) == ESP_OK);
    CHECK(!fake.supply_on && !fake.coil_on && !manager.status.power_asserted);
    CHECK(control_manager_tick(&manager, 80000) == ESP_OK);
    CHECK(submit_power(&manager, 3, 179000, true, false) == ROBOT_COMMAND_INVALID_STATE);
    CHECK(submit_power(&manager, 4, 180000, true, false) == ROBOT_COMMAND_ACCEPTED);
    CHECK(control_manager_tick(&manager, 190000) == ESP_OK);
    fake.power_good = false;
    CHECK(control_manager_tick(&manager, 200000) == ESP_OK);
    CHECK(manager.status.faults & ROBOT_FAULT_SOL_PG);
    CHECK(!fake.supply_on && !fake.coil_on);
}

static void test_solenoid_diagnostics_and_safety_stop(void)
{
    fake_actuators_t fake;
    robot_control_config_t config = control_manager_default_config();
    control_manager_t manager = make_manager(&fake, config);
    CHECK(submit_power(&manager, 1, 1000, true, true) == ROBOT_COMMAND_ACCEPTED);
    CHECK(control_manager_tick(&manager, 251000) == ESP_OK);
    CHECK(manager.status.faults & ROBOT_FAULT_SOL_PG);
    CHECK(!fake.supply_on && !fake.coil_on);

    manager = make_manager(&fake, config);
    CHECK(submit_power(&manager, 1, 1000, true, true) == ROBOT_COMMAND_ACCEPTED);
    fake.power_good = true;
    CHECK(control_manager_tick(&manager, 10000) == ESP_OK);
    CHECK(fake.supply_on && !fake.coil_on);
    CHECK(control_manager_tick(&manager, 10010000) == ESP_OK);
    CHECK(!fake.supply_on && !fake.coil_on);

    manager = make_manager(&fake, config);
    CHECK(submit_power(&manager, 1, 1000, true, true) == ROBOT_COMMAND_ACCEPTED);
    robot_command_t estop = command(ROBOT_CMD_ESTOP, 2, 2000);
    CHECK(control_manager_submit(&manager, &estop, 2000) == ROBOT_COMMAND_ACCEPTED);
    CHECK(!fake.supply_on && !fake.coil_on);
}

static void test_solenoid_control_timeout(void)
{
    fake_actuators_t fake;
    robot_control_config_t config = control_manager_default_config();
    config.command_timeout_ms = 100;
    control_manager_t manager = make_manager(&fake, config);
    robot_actuator_ops_t ops = manager.actuator_ops;
    ops.sol_pulse_ms = 200;
    ops.sol_cooldown_ms = 100;
    CHECK(control_manager_init(&manager, &config, &ops, &fake) == ESP_OK);
    CHECK(control_manager_boot_complete(&manager) == ESP_OK);
    CHECK(submit_power(&manager, 1, 1000, true, false) == ROBOT_COMMAND_ACCEPTED);
    fake.power_good = true;
    CHECK(control_manager_tick(&manager, 10000) == ESP_OK);
    CHECK(fake.coil_on);
    CHECK(control_manager_tick(&manager, 101000) == ESP_OK);
    CHECK(!fake.supply_on && !fake.coil_on);

    CHECK(control_manager_tick(&manager, 110000) == ESP_OK);
    robot_command_t disarm = command(ROBOT_CMD_DISARM, 3, 250000);
    CHECK(control_manager_submit(&manager, &disarm, 250000) == ROBOT_COMMAND_ACCEPTED);
    robot_command_t queued_start = command(ROBOT_CMD_POWER_CONTROL, 2, 249000);
    queued_start.value.power.asserted = true;
    CHECK(control_manager_submit(&manager, &queued_start, 250001) == ROBOT_COMMAND_INVALID_STATE);
    CHECK(!fake.supply_on && !fake.coil_on);
}

int main(void)
{
    test_arm_gate_and_mix();
    test_ramp_and_reverse_deadtime();
    test_expiry_sequence_and_estop_priority();
    test_watchdog_and_fault_clear();
    test_servo_range_slew_and_disable();
    test_solenoid_pulse_and_cooldown();
    test_solenoid_diagnostics_and_safety_stop();
    test_solenoid_control_timeout();
    if (failures) { fprintf(stderr, "%d assertion(s) failed\n", failures); return 1; }
    puts("control-core host tests: PASS");
    return 0;
}
