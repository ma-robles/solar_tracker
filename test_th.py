from __future__ import print_function
import solarMov
import time
class solar(solarMov.Loop):
    def neg_step(self,):
        print('neg:',self.pos)

    def pos_step(self,):
        print('pos:',self.pos)

print('iniciando...')
condition = True
moveX = solar(1,10)
moveY = solar(1,10)

moveX.start()
moveY.start()
key = 0
while key != 'q':
    key = raw_input('movimiento: ')
    if key != 'q':
        mov = int(key)
        moveX.put(mov)
        moveY.put(mov)
moveX.condition = False
moveY.condition = False
#wait for finish threads
time.sleep(1)
print ('god bye')
