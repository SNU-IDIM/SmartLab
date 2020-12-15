# SMART LAB
# Editor : Kyu-Wha Lee
# version : 0.0.1
# date : 2020.12.13
import pandas as pd
import numpy as np

ASCENDING_ORDER = 1
DESCENDING_ORDER = 2

class DoE(object):
    def __init__(self, info):
        self.factors = info[0]
        self.info = info[1]
        self.numberOfFactors = self.info['Factors']
        self.numberOfExperiments = self.info['Number']
        self.typeOfExperiment = self.info['Type']
        self.typeOfDoE = self.info['DoE']
        print ('Number Of Factors : ' , self.numberOfFactors)
        print ('Number Of Experiment : ' , self.numberOfExperiments)
        print ('Type Of Experiment : ' , self.typeOfExperiment)
        print ('Type Of DoE : ' , self.typeOfDoE)

        self.listFactors = list(self.factors)
        self.sortedFactors = sorted(self.factors)
        self.df = pd.DataFrame(columns = self.listFactors, dtype = int)

    def __len__(self):
        return len(self.df.count())

    def __setitem__(self):
        pass

    def __getitem__(self):
        pass

    def insert(self, data):
        if(sorted(data) == self.sortedFactors):
            self.df = self.df.append(data, ignore_index = True)
            # print(self.df)
        else:
            print('Wrong Data Input')

    def search(self, factor, value):
        if(factor in self.sortedFactors):
            return self.df.loc[self.df[factor] == value]
        else:
            print('No factor in Dataframe')

    def sort(self, factor, condition = ASCENDING_ORDER):
        if(factor in self.sortedFactors):
            if condition == ASCENDING_ORDER:
                return self.df.sort_values(by = factor, ascending = True)
            elif condition == DESCENDING_ORDER:
                return self.df.sort_values(by = factor, ascending = False)
        else:
            print('No factor in Dataframe')


if __name__ == '__main__':

    # Two Factors(Thickness, feedrate) and Two DOF EXPERIMENT
    info = [{'NUMBER', 'THICKNESS', 'FEEDRATES'}, {'Number' : 4, "Factors" : 2, "Type" : 'Tensile Test', 'DoE' : '2DOF'}] #depends on the initial setting by user // need to update
    a = DoE(info)
    FDM = {}
    # Result of Test Designer
    FDM[0] = {'NUMBER' : 1, 'THICKNESS' : 0.2, 'FEEDRATES' : 1}
    FDM[1] = {'NUMBER' : 2, 'THICKNESS' : 0.4, 'FEEDRATES' : 1}
    FDM[2] = {'NUMBER' : 3, 'THICKNESS' : 0.2, 'FEEDRATES' : 2}
    FDM[3] = {'NUMBER' : 4, 'THICKNESS' : 0.4, 'FEEDRATES' : 2}

    # save data
    a.insert(FDM[0])
    a.insert(FDM[1])
    a.insert(FDM[2])
    a.insert(FDM[3])

    print(a.search('NUMBER', 2)) #search number of experiment = 2
    print(a.search('THICKNESS', 0.2)) #serach thickness =2

    print(a.sort('NUMBER', DESCENDING_ORDER)) #sort by number with descending order
    print(a.sort('THICKNESS', ASCENDING_ORDER)) #sort by thickness with ascending order

    print(a.df)
