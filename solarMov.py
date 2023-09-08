#library for movements
from __future__ import print_function
from threading import Thread
from Queue import Queue
from time import sleep

class Move():
    #must extend with step method
    def move(self, distance, step_size=1):
        self.pos = abs(distance)
        while self. pos > 0:
            self.step(distance)
            self.pos -= step_size

    def step(self,):
        pass

class Step():
    #must extend with neg_step and pos_step
    def neg_step(self,):
        print(-self.pos)

    def pos_step(self,):
        print(self.pos)

    def step(self,direction):
        if direction<0:
            self.neg_step()
        if direction>0:
            self.pos_step()


class Loop(Thread,Step,Move):
    '''
    create a thread that realize movement
    indicated by mov_reg
    step_size: step size for every movement
    max_mov: max steps by loop
    write movement value with self.put(value) 
    end loop with self.condition=True
    '''
    def __init__(self, step_size, max_mov):
        Thread.__init__(self, )
        self.daemon =  True
        self.step_size = step_size
        self.max_mov = max_mov
        self.mov_reg = Queue(maxsize=1)
        self.pos = 0
        self.condition = True
        self.put(0)

    def put(self, mov):
        if self.mov_reg.qsize()!=0:
            self.mov_reg.get()
        self.mov_reg.put(mov)

    def run(self):
        mov_reg = 0
        #repeat while condition 
        while self.condition != False:
            step_size = self.step_size
            max_mov = self.max_mov
            if mov_reg == 0 or self.mov_reg.qsize()!=0:
                mov_reg = self.mov_reg.get()
            if abs(mov_reg)<max_mov:
                self.move(mov_reg, step_size)
                mov_reg = 0
            else:
                if mov_reg<0:
                    max_mov = -max_mov
                self.move(max_mov, step_size)
                mov_reg-= max_mov

    #extend this method
    #def move(self,):
    #    print('extend')
