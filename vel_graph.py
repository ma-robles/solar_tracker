import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
import ext_data 
import matplotlib.dates as dates

filenames=[
    'test/20140527.cca',
    'test/20140522.cca',
    'test/20140517.cca',
    'test/20140512.cca',
    'test/20140507.cca',
    'test/20140502.cca',
    ]
for filename in filenames:
    data = ext_data.file2cols(filename)
    mydata = ext_data.col2float(data[-2])
    date = ext_data.col2date(data[1])
    ax = plt.subplot(1,1,1)
    plt.plot_date(date,mydata,'-')
    #plt.legend([filename])
    myformat = dates.DateFormatter('%H:%M')
    ax.xaxis.set_major_formatter(myformat)
    plt.xticks(rotation='vertical')
    plt.ylabel('[degrees/s]')
    #filename = filename.rsplit('.')[0]+'.png'
    #plt.savefig(filename)
plt.legend(filenames)
plt.savefig('test/graphs2.png')
