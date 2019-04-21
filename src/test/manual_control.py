#!/usr/bin/env python
# -*- coding: utf-8 -*- 

# Copyright (c) 2017 co=driver.ai
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a real toy car with a keyboard.

"""
Welcome to CheYou toy car  manual control.

Use ARROWS or WASD keys for control.

    W            : throttle up 
    S            : throttle down
    AD           : steer
    Space        : hard-brake

    ESC          : quit
"""

from __future__ import print_function

import os
import sys



# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import datetime
import logging
import math
import random

try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


START_THROTTLE = 15
MAX_THROTTLE = 18
STOP_THROTTLE = 12

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self):
        self._steer_cache = 0.0
        self.throttle = 0.0
        self.steer = 0.0

    def _parse_vehicle_keys(self, keys, milliseconds):
        throttle_increment = 1e2 * milliseconds
        #油门设置为0，玩具车电机将停转，直接停车
        if keys[K_SPACE]:
            self.throttle=0
            

        if keys[K_UP] or keys[K_w]:
            #如果油门绝对值小于STOP_THROTTLE，意味着玩具车电机已经停转，需要启动的话，必须给一个大的油门START_THROTTLE
            if abs(self.throttle) < STOP_THROTTLE:
                self.throttle = START_THROTTLE
            else:
                self.throttle += throttle_increment
                self.throttle = min(self.throttle,MAX_THROTTLE)  #限制一个正向的最大的油门值
              
        if keys[K_DOWN] or keys[K_s]:
            #如果油门绝对值小于STOP_THROTTLE，意味着玩具车电机已经停转，需要启动的话，必须给一个大的油门START_THROTTLE
            if abs(self.throttle) < STOP_THROTTLE:
                self.throttle = -START_THROTTLE
            else:
                self.throttle -= throttle_increment
                self.throttle = max(self.throttle,-MAX_THROTTLE)  #限制一个反向的最大的油门值        
                
            
        steer_increment = 5e-2 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
            
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self.steer = round(self._steer_cache, 1)


    def parse_events(self,clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
    

        self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
        
        #pub_control(self.throttle,self.steer)
        print(self.throttle, self.steer)
        
    @staticmethod

    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop():
    pygame.init()
    pygame.font.init()
    
    try:
        controller = KeyboardControl()
        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(clock):
                return
    finally:
        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    try:
        game_loop()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':
    main()
