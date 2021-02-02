# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 10:11:48 2020

@author: Administrator
"""

interp_aci = np.zeros(len(XS_data), dtype=float)
print("we made it into interpolation") 
print(XS_data)
# print(type(float(XS_data)))
for i in range(len(XS_data)):
    print('value:', XS_data[i])
    print(type(XS_data[i]))
    print('i:', i)
    if XS_data[i] >= 0:
        print('in if')
        interp_aci[i] = rest_pos_aci[i] + \
            XS_data[i] * (max_pos_aci[i] - rest_pos_aci[i]) #Take the middle value -rest and find the diff with Max then add Rest
        print(XS_data[i])
    else:                                                    #E.G. 1024 - 900 = 124 * .80 = 99.2 + 900 = 999.2 
        print('in else')
        interp_aci[i] = rest_pos_aci[i] - \
            abs(XS_data[i] * (rest_pos_aci[i] - min_pos_aci[i])) #Take the middle value -rest and find the abs diff with Min then add Min again
        print(XS_data[i])
    print("we made it through interpolation")            # E.G. 500 - 900 = abs(400 * -.80) = 900-320  = 580