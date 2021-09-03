#from __future__ import absolute_import
#from __future__ import print_function

import os
import sys
import math
import json
import random
import numpy as np
import traceback

from scipy.stats import lognorm
from scipy.stats import uniform
from scipy.stats import weibull_min
from scipy.stats import gamma


################################
# Risk-based Predictive Model developed using NGSIM data
################################
#lag crash risk prediction   #revised the function
def risk_lag_prior(Speed_Merge,Acce_Merge,Remaining_Distance,Acce_lag,Gap_lag,Speed_dif):
    beta=[-0.648166,0.291651,-2.67226,0.010325,2.736206,-1.9484,3.314949] #beta=[intercept,variable1,variable2,...] 
    X=[1,Speed_Merge,Acce_Merge,Remaining_Distance,Acce_lag,Gap_lag,Speed_dif]
    Pr_lag_prior=1/(1+np.exp(-np.dot(beta,X)))
    print("Pr_lag_prior",Pr_lag_prior)
    return Pr_lag_prior

def lag_conflict_likelihood(Gap_lag):
    #weibull cdf
    c=1.177935
    loc=0
    scale=15.060868 
    x_upper=Gap_lag+0.5
    x_lower=Gap_lag-0.5
    Pr_gap_lag_conflict=weibull_min.cdf(x_upper,c,loc,scale)-weibull_min.cdf(x_lower, c,loc,scale)
    return Pr_gap_lag_conflict
    
def lag_nonconflict_likelihood(Gap_lag):
 
    shape = 3.82145718
    rate = 0.06710455
    #gamma cdf
    loc=0
    scale=1/rate
    x_upper=Gap_lag+0.5
    x_lower=Gap_lag-0.5
    Pr_gap_lag_nonconflict=gamma.cdf(x_upper,shape,loc,scale)-gamma.cdf(x_lower,shape,loc,scale)
    return Pr_gap_lag_nonconflict



def risk_lag_posterior(Gap_lag,Pr_lag_prior,Pr_gap_lag_nonconflict,Pr_gap_lag_conflict):

    #print ('1-Pr_lag_prior  = ', 1-Pr_lag_prior)
    #print ('Pr_gap_lag_conflict * Pr_lag_prior = ', Pr_gap_lag_conflict * Pr_lag_prior)
    #print ('((Pr_gap_lag_conflict * Pr_lag_prior) + Pr_gap_lag_nonconflict *(1-Pr_lag_prior)) = ', ((Pr_gap_lag_conflict * Pr_lag_prior) + Pr_gap_lag_nonconflict *(1-Pr_lag_prior)))
    #print ('Pr_lag_prior * Pr_gap_lag_conflict = ', Pr_lag_prior * Pr_gap_lag_conflict)

    denom = ((Pr_gap_lag_conflict * Pr_lag_prior) + Pr_gap_lag_nonconflict *(1-Pr_lag_prior))

    if(denom == 0):
        return 1.0
    
    Pr_lag_posterior= (Pr_lag_prior * Pr_gap_lag_conflict)/denom
    #print ('Pr_lag_posterior: ', Pr_lag_posterior)

    #print ('')
    return Pr_lag_posterior


def risk_lead_prior(Acce_Merge,Remaining_Distance,Speed_lead,Acce_lead,Gap_lead):
    beta=[-0.871,-1.257,0.029,-0.034,0.451,-0.301]
    X=[1,Acce_Merge,Remaining_Distance,Speed_lead,Acce_lead,Gap_lead]
    Pr_lead_prior=1/(1+np.exp(-np.dot(beta,X)))
    return Pr_lead_prior

def lead_conflict_likelihood(Gap_lead):
    #uniform cdf
    loc= -4.996666
    scale=52.099515+4.996666
    x_upper=Gap_lead+0.5
    x_lower=Gap_lead-0.5
    Pr_gap_lead_conflict=uniform.cdf(x_upper,loc,scale)-uniform.cdf(x_lower,loc,scale)
    return Pr_gap_lead_conflict
    
def lead_nonconflict_likelihood(Gap_lead):
    #weibull cdf
    c= 1.609829 
    loc=0
    scale=49.765264
    x_upper=Gap_lead+0.5
    x_lower=Gap_lead-0.5
    Pr_gap_lead_conflict=weibull_min.cdf(x_upper,c,loc,scale)-weibull_min.cdf(x_lower, c,loc,scale)
    return Pr_gap_lead_conflict


def risk_lead_posterior(Gap_lead,Pr_lead_prior,Pr_gap_lead_nonconflict,Pr_gap_lead_conflict):
    denom = (Pr_gap_lead_conflict*Pr_lead_prior+Pr_gap_lead_nonconflict*(1-Pr_lead_prior))
    if(denom == 0):
        return 1.0    
    Pr_lead_posterior=Pr_lead_prior*Pr_gap_lead_conflict/(Pr_gap_lead_conflict*Pr_lead_prior+Pr_gap_lead_nonconflict*(1-Pr_lead_prior))
    return Pr_lead_posterior



def safety_distance_min(Speed_Merge,Speed_lead):
    b_av=4.6
    b_lead=4.2
    tau_av=0.9
    S_min=Speed_Merge*tau_av+1/2*np.square(Speed_Merge)/b_av-1/2*np.square(Speed_lead)/b_lead
    return S_min



 
