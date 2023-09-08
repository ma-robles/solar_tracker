from __future__ import print_function
import solarMov
import time

class newLoop(solarMov.Loop):
    def neg_step(self,):
        print('new neg',self.pos)

    def pos_step(self,):
        print('new pos',self.pos)

print('iniciando')
movX = newLoop(1,10)
movX.start()
movX.put(100)
time.sleep(10)
movX.condition = False
print('saliendo')

