#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import logging
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler

def user_input():
 arr = []
 given_input = ''

 while len(arr) < 6:
   try:
      given_input = float(input('Enter an number: '))
      arr.append(given_input)
   except ValueError :
      print('please enter a number')
      continue
 return arr  

if __name__ == "__main__": 
    os.system('python3 file2.py') 
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(message)s')
    path = (r"sample.json")
    event_handler = LoggingEventHandler()
    observer = Observer()
    observer.schedule(event_handler, path, recursive=True) 
    observer.start()  #for starting the observer thread
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()       

