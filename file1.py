#!/usr/bin/env python3

from file2 import *

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
user_input()   
print(input(file2.param))        

