import logging
from datetime import datetime

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

system_time = datetime.now()
time_str = system_time.strftime('%H:%M:%S.%f')[:-3]

print(f"Current system time: {time_str}")
