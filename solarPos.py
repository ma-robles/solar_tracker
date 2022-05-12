'''Equations based on NOAA web page'''
#autor: Miguel Angel Robles Roldan
from __future__ import division
from datetime import datetime
from datetime import timedelta
import math
from math import pi
from math import cos
from math import sin
from math import acos

class solarPos():
    def __init__(self, lat, lon, dtime, tzone):
        self.lat = math.radians(lat)
        self.lon = math.radians(lon)
        self.dtime = dtime
        self.tzone = tzone
        self.calculate()

    def calculate(self,):
        self.w = self.get_w(self.dtime)
        self.eq_time= self.get_eqtime(self.w)
        self.hourA = self.get_hourA(self.dtime, self.w)
        self.decl = self.get_decl(self.w)
        self.zen_cos = self.get_zen_cos(
                self.lat,
                self.decl,
                self.hourA,
                )
        self.el = (pi/2)-acos(self.zen_cos)
        self.az_cos = self.get_az_cos(
                self.el,
                self.decl,
                self.lat,
                )
        self.az = acos(self.az_cos)
        
        if self.hourA>0:
            self.az = 2*pi-self.az
    
    def get_w(self, dtime):
        #Get fractional year in radians
        #fractional day is a convertion 
        #1 year = 2pi rad
        #initial date
        i_day = datetime(self.dtime.year,1,1)
        #seconds by year
        year = 365*24*60*60
        #calculate
        date = dtime-i_day
        date= date.total_seconds()
        return 2*pi*date/year

    def get_eqtime(self, w):
        '''estimate equation of time in minutes'''
        eqtime = 7.5e-5+1.868e-3*cos(w)
        eqtime -= 3.2077e-2*sin(w)
        eqtime -= 1.4615e-2*cos(2*w)
        eqtime -= 4.0849e-2*sin(2*w)
        return 229.18*eqtime

    def get_decl(self, w):
        '''calculate solar declination angle in radians'''
        decl = (6.918e-3)-0.399912*cos(w)
        decl += (7.0257e-2)*sin(w)
        decl -= (6.758e-3)*cos(2*w)
        decl += (9.07e-4)*sin(2*w)
        decl -= (2.697e-3)*cos(3*w)
        decl += (1.48e-3)*sin(3*w)
        return decl

    def get_hourA(self, dtime, w):
        '''
        return hour angle in radians
        dtime is datetime object 
        '''
        #time offset[seconds]
        t_offset=self.eq_time
        t_offset+=4*(math.degrees(self.lon))
        t_offset-=60*self.tzone
        t_offset*=60
        i_time= datetime(dtime.year, dtime.month, dtime.day)
        #true solar time[seconds]
        ts_time= (dtime-i_time).total_seconds()
        ts_time+= t_offset
        #hour angle[rad]
        return (ts_time*2*pi/86400)-pi

    def get_zen_cos(self, lat, decl, hourA):
        '''calculate cos(phi)
        lat, decl and hourA in radians
        '''
        zen_cos = sin(lat)*sin(decl)
        zen_cos += cos(lat)*cos(decl)*cos(hourA)
        return zen_cos

    def get_az_cos(self, elev, decl, lat, ):
        '''calculate azimuth
        return cos(180-q)
        '''
        az_cos = sin(decl)-sin(lat)*sin(elev)
        az_cos /= cos(lat)*cos(elev)
        #other way
        #az_cos = sin(decl)*cos(lat)
        #az_cos -= cos(hourA)*cos(decl)*sin(lat)
        #az_cos /= cos(elev)
        return az_cos
