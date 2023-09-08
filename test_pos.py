from __future__ import print_function
from datetime import datetime
import solarPos
import math
from datetime import timedelta
import pickle
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt

def derivate(values):
    i = 1
    der=[]
    while i<len(values):
        der.append(values[i]-values[i-1])
        i+=1
    return der

mydate =datetime(2015, 5, 20)
#dtime = timedelta(days=5)
#enddate=datetime(2015, 9, 2)
dates=[mydate]
azimuth =[]
elevation =[]
ls_time=[]
   
for mydate in dates:
    #init time
    init_time=timedelta(hours=8)
    #end time
    end_time= timedelta(hours=18, minutes=0)
    #calculate
    track_time = mydate+init_time
    end = mydate+end_time
    sol = solarPos.solarPos(19.4, -99.149999, track_time, -6)
    ls_time=[track_time]
    azimuth=[math.degrees(sol.az)]
    elevation=[math.degrees(sol.el)]
    inter = timedelta(minutes=1)
    while(track_time<end):
        track_time += inter
        sol=solarPos.solarPos(19.4, -99.149999, track_time, -6)
        ls_time.append(track_time)
        azimuth.append(math.degrees(sol.az))
        elevation.append(math.degrees(sol.el))
    #Vel = d_el/inter.total_seconds()
    vel_az=derivate(azimuth)
    flag_change=False
    for i in range(len(vel_az)):
        if vel_az[i]>180:
            flag_change = True
        if flag_change==True:
            azimuth[i+1]-=360

    vel_az=derivate(azimuth)
    max_az=vel_az.index(max(vel_az))
    min_az=min(vel_az)
    vel_el=derivate(elevation)
    acel_az=derivate(vel_az)
    acel_el=derivate(vel_el)


    #with open('graphs/test.cca','w') as dfile:
    #    pickle.dump(ls_az,dfile)
    ax = plt.subplot(3,1,1)
    myformat=mpl.dates.DateFormatter('%H:%M')
    ax.xaxis.set_major_formatter(myformat)
    plt.title(mydate.strftime('%d %B %Y'))
    plt.plot_date(ls_time,azimuth,'-')
    plt.plot_date(ls_time,elevation,'-')
    plt.axvline(ls_time[max_az],ls='dashed', alpha=0.5)

    ax = plt.subplot(3,1,2)
    ax.xaxis.set_major_formatter(myformat)
    plt.plot_date(ls_time[:len(vel_az)],vel_az,'-')
    plt.plot_date(ls_time[:len(vel_el)],vel_el,'-')
    plt.axvline(ls_time[max_az],ls='dashed', alpha=0.5)
    plt.axhline(min_az,ls='dashed',alpha=0.5,)
    ax.grid(True)

    ax = plt.subplot(3,1,3)
    ax.xaxis.set_major_formatter(myformat)
    plt.plot_date(ls_time[:len(acel_az)],acel_az,'-')
    plt.plot_date(ls_time[:len(acel_el)],acel_el,'-')
    plt.axvline(ls_time[max_az],ls='dashed', alpha=0.5)
    #plt.xticks(rotation='vertical')
    plt.savefig('graphs/test.png')
    #print('Archivo generado')
    print('proceso terminado')
