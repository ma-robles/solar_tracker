import matplotlib.dates as dates

def file2cols(myfile, sep =' '):
    dataCol=[]
    with open(myfile, 'r') as f:
        n=0
        for line in f:
            words = line.split()
            for i,ncol in enumerate(words):
                if n==0:
                    dataCol.append([])
                dataCol[i].append(ncol)
            n+=1
    return dataCol

def col2float(data):
    temp =[]
    for d in data:
        temp.append(float(d))
    return temp

def col2date(data):
    myformat = dates.strpdate2num('%H:%M:%S')
    mydate = []
    for d in data:
        mydate.append(myformat(d))
    return mydate
