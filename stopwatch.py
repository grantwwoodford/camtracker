from datetime import datetime
from time import sleep

start = datetime.now()

def start_stopwatch():
    global start
    start = datetime.now()

def get_elapsedtime():
    endtime = datetime.now()

    # get milliseconds elapsed since timer was started
    return (endtime - start).total_seconds()*1000

#start_stopwatch()
#sleep(3)
#print get_elapsedtime()
