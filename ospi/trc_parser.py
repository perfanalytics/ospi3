# Read trc header - https://github.com/perfanalytics/pose2sim/blob/main/Pose2Sim/Utilities/trc_filter.py

import pandas as pd
import numpy as np
from numpy.linalg import norm
from scipy import signal

def read_trc(trc_path_in, filter, rotate_axis = False):

    with open(trc_path_in, 'r') as trc_file:
        header = [next(trc_file) for line in range(5)]
    frame_rate = int(float(header[2].split('\t')[0]))
    L_marker_name = header[3].split("\t",2)[2].split("\t\t\t")[:-1]
    # Read trc coordinates values
    trc_df = pd.read_csv(trc_path_in, sep="\t", skiprows=4)
    trc_df.rename(columns={'Unnamed: 0':'frame_number','Unnamed: 1':'time'},inplace=True)

    #redifining index
    trc_df.set_index(pd.MultiIndex.from_frame(trc_df[['frame_number','time']]),inplace=True)
    trc_df.drop('frame_number', inplace=True, axis=1)
    trc_df.drop('time', inplace=True, axis=1)

    if header[2].split("\t")[4]=="mm": #convertit les coordonées de mm à m
        trc_df.iloc[:,:]=trc_df.iloc[:,:]/1000
    elif header[2].split("\t")[4]!="m":
        print('error units - check parsing trc')

    if rotate_axis:
        column_names = trc_df.columns
        for i in range(len(L_marker_name)):
            trc_df.rename(columns={f"Y{i+1}":f"Z{i+1}",f"Z{i+1}":f"Y{i+1}"},inplace=True)
            trc_df[f"Y{i+1}"]=-trc_df[f"Y{i+1}"]
            trc_df.rename(columns={f"X{i+1}":f"Y{i+1}",f"Y{i+1}":f"X{i+1}"},inplace=True)
            trc_df[f"X{i+1}"]=-trc_df[f"X{i+1}"]
        trc_df = trc_df[column_names] #reordering columns
    
    #filtering data TODO
    if filter:
        for coor in trc_df.columns:
            trc_df[coor]=butterworth_filter_1d(list(trc_df[coor]))
            #trc_df[coor]=simple_filter(list(trc_df[coor]))

    #aggregate data by marker
    trc_df_agg = pd.DataFrame(columns=L_marker_name,index=trc_df.index) 

    for j,[(frame_number,t),row] in enumerate(trc_df.iterrows()):
        for i in range(len(L_marker_name)):
            trc_df_agg.iloc[j][i] = [row[3*i], row[3*i+1], row[3*i+2]]
    return trc_df_agg

def adapte_to_marker_set(trc_df, markerSet):
    """
    Check and remove marker that are not in both set
    """
    L_marker_trc = list(trc_df.columns) # list of marker names in the trc_df
    L_marker_set = np.array(markerSet,dtype=object)[:,0].tolist() # list of marker names in the marker set

    pointage = [0 for i in range(len(L_marker_set))]

    for marker in L_marker_trc:
        i=0
        while i<len(L_marker_set):
            if L_marker_set[i]==marker:
                pointage[i]=1
                break
            i+=1
        if i==len(L_marker_set):
            trc_df.drop(marker, inplace=True, axis=1)
            print(marker,"marker from .trc doesn't match any marker from markerset")

    if sum(pointage)<len(pointage):
        print("\nSome markers of the markerset are not in the trc file : ")
        for i in reversed(range(len(pointage))):
            if pointage[i]==0:
                print(L_marker_set[i], "is missing.")
                del L_marker_set[i]

    #reordering markers according to markerset order
    trc_df = trc_df[L_marker_set] 
    return trc_df
        
def simple_filter(col):
    pas = []
    for i in range(1,len(col)-1):
        if (norm(col[i-1]-col[i])>100*norm(col[i-1]-col[i+1]) and norm(col[i+1]-col[i])>100*norm(col[i-1]-col[i+1])):
            col[i]=(col[i-1]+col[i+1])/2
    return col

def butterworth_filter_1d(col): 
    '''
    1D Zero-phase Butterworth filter (dual pass)
    INPUT:
    - col: Pandas dataframe column
    - args: dictionary of pass_type, order, cut_off_frequency, frame_rate
    OUTPUT
    - col_filtered: Filtered pandas dataframe column
    '''

    butterworth_filter_type = 'low' 
    butterworth_filter_order = 4 
    butterworth_filter_cutoff = 6 
    frame_rate = 200

    b, a = signal.butter(butterworth_filter_order/2, butterworth_filter_cutoff/(frame_rate/2), butterworth_filter_type, analog = False) 
    col_filtered = signal.filtfilt(b, a, col)
    
    return col_filtered
