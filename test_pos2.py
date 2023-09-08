import solarPos
from math import degrees
import datetime

mytime = datetime.datetime(year=2015, month=8, day =4, hour=8)
dtime= datetime.timedelta (minutes=30)
for i in range(30):
    dtime=datetime.timedelta(hours=i*0.25)
    atime = mytime+dtime
    sol = solarPos.solarPos(19.4, -99.1499, atime, -6)
    az = degrees(sol.az)

    print(atime,az,degrees(sol.el))
