#!/usr/bin/python

import socket
from stopwatch import *
from colourtracker import *
from thread import *

HOST = '0.0.0.0'   # Symbolic name meaning all available interfaces
PORT = 3030 # Arbitrary non-privileged port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print 'Socket created'

#Bind socket to local host and port
try:
    s.bind((HOST, PORT))
except socket.error as msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()

print 'Socket bind complete'

#Start listening on socket
s.listen(10)
print 'Socket now listening'

#Send message
def sendMessage(conn, message):
    #Send some data to remote server
    message = message + "\r\n"
    #print message

    try :
        #Set the whole string
        conn.sendall(message)
    except socket.error:
        #Send failed
        print 'Send failed'
        sys.exit()

#Position sending function
def sendPosition(conn, data):
    commandStringArray = data.split(':')

    if len(commandStringArray) != 2:
        sendMessage(conn, "P:invalid second argument")
        return

    timestamp = int(commandStringArray[1])

    # fetches position for specific timestamp, then calibrates the point before senting to client
    cur_position = apply_calibration(take_interpolated_value(timestamp))

    sendMessage(conn, formatted_sending_position(timestamp, cur_position))

def setup_calibration():
    reset_calibration()

#Handle timesync
def syncTime(conn, data):
    commandStringArray = data.split(':')

    if len(commandStringArray) != 2:
        sendMessage(conn, "S:invalid second argument")
        print "S:invalid second argument:" + data
        return

    # reset calibration
    setup_calibration()

    if commandStringArray[1] == "START":
        # start timer
        start_stopwatch()

        # reset list of tracked positions
        reset_positions()

        sendMessage(conn, "S:SERVER_STARTED")

        response = conn.recv(1024).rstrip()
        trip_elapse_time = get_elapsedtime()

        #print response

        # send time it took to do round trip, so client can offset own timer
        sendMessage(conn, "S:DELAY_PERIOD:"+str(int(trip_elapse_time)))

        print "timesync complete"
    else:
        print "Unrecognized time sync command."
        print commandStringArray[1]


#Function for handling connections. This will be used to create threads
def clientthread(conn):
    #infinite loop so that function do not terminate and thread do not end.
    while True:

        #Receiving from client
        data = conn.recv(1024).rstrip()
        print data

        if len(data) == 0:
            break

        if len(data) > 0 and data[0] == "P":
            sendPosition(conn, data)
        elif len(data) > 0 and data[0] == "S":
            syncTime(conn, data)
        else:
            print "Error! " + data
            sendMessage(conn, "E:Unrecognised command")

    #came out of loop
    conn.close()

def main(argv):
    print argv
    # start colour tracker
    start_new_thread(trackingthread, (argv,))

    #now keep talking with the client
    while 1:
        #wait to accept a connection - blocking call
        conn, addr = s.accept()
        #print 'Connected with ' + addr[0] + ':' + str(addr[1])

        #start new thread takes 1st argument as a function name to be run, second is the tuple of arguments to the function.
        start_new_thread(clientthread ,(conn,))

    s.close()

if __name__ == "__main__":
   main(sys.argv[1:])