import numpy as np

def mimic_training(SS):
    SS['kin'][:] = SS['train_seq'][:, SS['train_iter']]
    SS['train_fid'].write(np.r_[SS['cur_time'], SS['feat'], SS['kin']].astype('single'))
    if SS['train_iter'] == SS['train_seq'].shape[1]-1: # finished training
        SS['train_iter'] = None
        SS['train_fid'].close()
        print('Training finished')
        SS['train_kf_phase'] = 'StartChanSel'
    else: # haven't finished training
        SS['train_iter'] += 1
    return SS
