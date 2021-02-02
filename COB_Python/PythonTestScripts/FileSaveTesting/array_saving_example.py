# -*- coding: utf-8 -*-
"""
Created on Fri Sep 11 11:20:38 2020

@author: Administrator
"""
import time



time.sleep(1)


## using np.save, which should be platform independent (but is ~5x slower than tofile)
timestr = time.strftime('%Y%m%d-%H%M%S')
train_fid = open('trainKDF_' + timestr + '.kdf', 'wb')

t0 = time.clock()
np.save(train_fid, mLFP.astype('single'), allow_pickle=False)
np.save(train_fid, mLFP.astype('single'), allow_pickle=False)
np.save(train_fid, mLFP.astype('single'), allow_pickle=False)
t1 = time.clock()

train_fid.close()
print(t1 - t0)

train = open('trainKDF_' + timestr + '.kdf', 'rb')

read_mLFP = np.load(train)

train.close()

time.sleep(1)


## using np.save, which should be platform independent
timestr = time.strftime('%Y%m%d-%H%M%S')
train_fid = open('trainKDF_' + timestr + '.kdf', 'wb')

t0 = time.clock()
np.save(train_fid, mLFP.astype('single'))
np.save(train_fid, mLFP.astype('single'))
np.save(train_fid, mLFP.astype('single'))
t1 = time.clock()

train_fid.close()
print(t1 - t0)

train = open('trainKDF_' + timestr + '.kdf', 'rb')

read_mLFP = np.load(train)

train.close()

time.sleep(1)

## using .tofile(), not platform independent
timestr = time.strftime('%Y%m%d-%H%M%S')
train_fid = open('trainKDF_' + timestr + '.kdf', 'wb')

t0 = time.clock()
mLFP.astype('single').tofile(train_fid)
mLFP.astype('single').tofile(train_fid)
mLFP.astype('single').tofile(train_fid)
t1 = time.clock()

train_fid.close()
print(t1 - t0)

train = open('trainKDF_' + timestr + '.kdf', 'rb')

read_mLFP = np.fromfile(train, dtype='single')

train.close()



#############################################################################
# other stuff we tried... doesn't work well with writing the raw data
        import array

        a = array.array('f')
        train_data = a.fromfile(train_fid, 3)
        train_data = train_fid.read()
        train_fid.close()
                np.load(train_fid)
