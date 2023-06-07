import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.family'] = 'cmu serif'
# use latex for font rendering
matplotlib.rcParams['text.usetex'] = True

SMALL_SIZE = 12
MEDIUM_SIZE = 14
BIGGER_SIZE = 16

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.core.problem import Problem
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.operators.repair.rounding import RoundingRepair
from pymoo.operators.sampling.rnd import IntegerRandomSampling
from pymoo.optimize import minimize

import os
import sys
import comtypes.client

import time

from threading import Thread
import subprocess

def split(a, n):
    k, m = divmod(len(a), n)
    return (a[i*k+min(i, m):(i+1)*k+min(i+1, m)] for i in range(n))

class MyProblem(Problem):

    def __init__(self,SapModel, SapObject, LIST_HE, LIST_IPE, ModelPath , Main_File_name , File_ext):

        self.SapModel = SapModel
        self.SapObject = SapObject
        self.LIST_HE = LIST_HE
        self.LIST_IPE = LIST_IPE
        self.ModelPath = ModelPath
        self.Main_File_name = Main_File_name
        self.File_ext = File_ext
        self.gen = 0

        n_var = 6

        xl = np.zeros((n_var,))
        xu = np.array([len(LIST_HE)-1,len(LIST_HE)-1,len(LIST_HE)-1,
                       len(LIST_IPE)-1,len(LIST_IPE)-1,len(LIST_IPE)-1])

        super().__init__(n_var=n_var, n_obj=1, n_ieq_constr=4, xl=xl, xu=xu, vtype=int)

    def _evaluate(self, x, out, *args, **kwargs):
        List_file_names =[]
        for ii in range(x.shape[0]):
            List_file_names.append(f'{self.ModelPath}{self.Main_File_name}-{ii}{self.File_ext}')

        List_file_names = list(split(List_file_names, 3))
        X = np.array_split(x, 3)

        global outF1 
        global outG1 
        global outF2 
        global outG2 
        global outF3
        global outG3

        outF1 = np.zeros(len(List_file_names[0]))
        outG1 = np.zeros((len(List_file_names[0]),4))
        outF2 = np.zeros(len(List_file_names[1]))
        outG2 = np.zeros((len(List_file_names[1]),4))
        outF3 = np.zeros(len(List_file_names[2]))
        outG3 = np.zeros((len(List_file_names[2]),4))

        def run1(SapModel, Files, X, OBJ):
            global outF1
            global outG1
            for jj,file_name in enumerate(Files):
                SapModel.File.OpenFile(OBJ.ModelPath + OBJ.Main_File_name + OBJ.File_ext)
                ret = SapModel.SetModelIsLocked(False)
                for kk in range(1,16): 
                    if kk in [1,4,7]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_HE[X[jj,0]])
                        # print(ret, self.LIST_HE[x[ii,0]])
                    elif kk in [2,5,8]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_HE[X[jj,1]])
                    elif kk in [3,6,9]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_HE[X[jj,2]])
                    elif kk in [10,11]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_IPE[X[jj,3]])
                    elif kk in [12,13]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_IPE[X[jj,4]])
                    elif kk in [14,15]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_IPE[X[jj,5]])
                        # print(ret, self.LIST_IPE[x[ii,0]])
                ret = SapModel.File.Save(file_name)
                ret = SapModel.File.OpenFile(file_name)
                ret = SapModel.Analyze.RunAnalysis()
                ret = SapModel.DesignSteel.StartDesign()
                ret = SapModel.Results.Setup.SetCaseSelectedForOutput("DEAD")
                [NumberResults, LoadCase, StepType, StepNum, Fx, Fy, Fz, Mx, My, Mz, gx, gy, gz, ret] = SapModel.Results.BaseReact()
                outF1[jj] = Fz[0]
                g_summary = np.zeros(16)
                g_NMM = np.zeros(16)
                # g_V = np.zeros(16)
                for kk in range(1,16): 
                    [NumberItems, FrameName, Ratio, RatioType, Location, ComboName, ErrorSummary, WarningSummary, ret] = SapModel.DesignSteel.GetSummaryResults(f'{kk}')
                    g_summary[kk-1] = Ratio[0]
                    [NumberItems, FrameName, Value, ret] = SapModel.DesignSteel.GetDetailResultsValue(f"{kk}", 0, 2, "TotalRatio")
                    g_NMM[kk-1] = Value[0]
                    # [NumberItems, FrameName, Value, ret] = self.SapModel.DesignSteel.GetDetailResultsValue(f"{jj}", 0, 3, "V2Ratio")
                    # g_V[jj-1] = Value[0]
                outG1[jj,0] = sum(np.max((g_NMM - 1, np.zeros(16)), axis=0))
                outG1[jj,1] = sum(np.max((g_summary - 1, np.zeros(16)), axis=0))
                # Vincoli geometrici
                outG1[jj,2] = max( X[jj,1] - X[jj,0] , 0)
                outG1[jj,3] = max( X[jj,2] - X[jj,1] , 0)
                

        def run2(SapModel, Files, X, OBJ):
            global outF2
            global outG2
            for jj,file_name in enumerate(Files):
                SapModel.File.OpenFile(OBJ.ModelPath + OBJ.Main_File_name + OBJ.File_ext)
                ret = SapModel.SetModelIsLocked(False)
                for kk in range(1,16): 
                    if kk in [1,4,7]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_HE[X[jj,0]])
                        # print(ret, self.LIST_HE[x[ii,0]])
                    elif kk in [2,5,8]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_HE[X[jj,1]])
                    elif kk in [3,6,9]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_HE[X[jj,2]])
                    elif kk in [10,11]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_IPE[X[jj,3]])
                    elif kk in [12,13]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_IPE[X[jj,4]])
                    elif kk in [14,15]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_IPE[X[jj,5]])
                        # print(ret, self.LIST_IPE[x[ii,0]])
                ret = SapModel.File.Save(file_name)
                ret = SapModel.File.OpenFile(file_name)
                ret = SapModel.Analyze.RunAnalysis()
                ret = SapModel.DesignSteel.StartDesign()
                ret = SapModel.Results.Setup.SetCaseSelectedForOutput("DEAD")
                [NumberResults, LoadCase, StepType, StepNum, Fx, Fy, Fz, Mx, My, Mz, gx, gy, gz, ret] = SapModel.Results.BaseReact()
                outF2[jj] = Fz[0]
                g_summary = np.zeros(16)
                g_NMM = np.zeros(16)
                # g_V = np.zeros(16)
                for kk in range(1,16): 
                    [NumberItems, FrameName, Ratio, RatioType, Location, ComboName, ErrorSummary, WarningSummary, ret] = SapModel.DesignSteel.GetSummaryResults(f'{kk}')
                    g_summary[kk-1] = Ratio[0]
                    [NumberItems, FrameName, Value, ret] = SapModel.DesignSteel.GetDetailResultsValue(f"{kk}", 0, 2, "TotalRatio")
                    g_NMM[kk-1] = Value[0]
                    # [NumberItems, FrameName, Value, ret] = self.SapModel.DesignSteel.GetDetailResultsValue(f"{jj}", 0, 3, "V2Ratio")
                    # g_V[jj-1] = Value[0]
                outG2[jj,0] = sum(np.max((g_NMM - 1, np.zeros(16)), axis=0))
                outG2[jj,1] = sum(np.max((g_summary - 1, np.zeros(16)), axis=0))
                # Vincoli geometrici
                outG2[jj,2] = max( X[jj,1] - X[jj,0] , 0)
                outG2[jj,3] = max( X[jj,2] - X[jj,1] , 0)
                

        def run3(SapModel, Files, X, OBJ):
            global outF3
            global outG3
            for jj,file_name in enumerate(Files):
                SapModel.File.OpenFile(OBJ.ModelPath + OBJ.Main_File_name + OBJ.File_ext)
                ret = SapModel.SetModelIsLocked(False)
                for kk in range(1,16): 
                    if kk in [1,4,7]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_HE[X[jj,0]])
                        # print(ret, self.LIST_HE[x[ii,0]])
                    elif kk in [2,5,8]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_HE[X[jj,1]])
                    elif kk in [3,6,9]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_HE[X[jj,2]])
                    elif kk in [10,11]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_IPE[X[jj,3]])
                    elif kk in [12,13]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_IPE[X[jj,4]])
                    elif kk in [14,15]:
                        ret = SapModel.FrameObj.SetSection(f"{kk}", OBJ.LIST_IPE[X[jj,5]])
                        # print(ret, self.LIST_IPE[x[ii,0]])
                ret = SapModel.File.Save(file_name)
                ret = SapModel.File.OpenFile(file_name)
                ret = SapModel.Analyze.RunAnalysis()
                ret = SapModel.DesignSteel.StartDesign()
                ret = SapModel.Results.Setup.SetCaseSelectedForOutput("DEAD")
                [NumberResults, LoadCase, StepType, StepNum, Fx, Fy, Fz, Mx, My, Mz, gx, gy, gz, ret] = SapModel.Results.BaseReact()
                outF3[jj] = Fz[0]
                g_summary = np.zeros(16)
                g_NMM = np.zeros(16)
                # g_V = np.zeros(16)
                for kk in range(1,16): 
                    [NumberItems, FrameName, Ratio, RatioType, Location, ComboName, ErrorSummary, WarningSummary, ret] = SapModel.DesignSteel.GetSummaryResults(f'{kk}')
                    g_summary[kk-1] = Ratio[0]
                    [NumberItems, FrameName, Value, ret] = SapModel.DesignSteel.GetDetailResultsValue(f"{kk}", 0, 2, "TotalRatio")
                    g_NMM[kk-1] = Value[0]
                    # [NumberItems, FrameName, Value, ret] = self.SapModel.DesignSteel.GetDetailResultsValue(f"{jj}", 0, 3, "V2Ratio")
                    # g_V[jj-1] = Value[0]
                outG3[jj,0] = sum(np.max((g_NMM - 1, np.zeros(16)), axis=0))
                outG3[jj,1] = sum(np.max((g_summary - 1, np.zeros(16)), axis=0))
                # Vincoli geometrici
                outG3[jj,2] = max( X[jj,1] - X[jj,0] , 0)
                outG3[jj,3] = max( X[jj,2] - X[jj,1] , 0)                

        threads = []

        t = Thread(target=run1, args=(SapModel[0], List_file_names[0], X[0], self))
        threads.append(t)
        t = Thread(target=run2, args=(SapModel[1], List_file_names[1], X[1], self))
        threads.append(t)
        t = Thread(target=run3, args=(SapModel[2], List_file_names[2], X[2], self))
        threads.append(t)

        # Start to evaluate time of analysis in parallel
        start = time.time()

        # Start all threads
        for tt in threads:
            tt.start()

        # Wait for all of them to finish
        for tt in threads:
            tt.join()

        # Evaluate time of analysis in parallel
        end = time.time()
        print('Elapsed time :',end - start,'s')

        # some code after execution of all bat files

        out["F"] = np.hstack( (outF1, outF2, outF3) )
        out["G"] = np.vstack( (outG1, outG2, outG3) )
        
        self.gen += 1
        print(f'gen {self.gen}\n','F=', out["F"], '\n', 'G=', out["G"], '\n\n')










Num_SAP2000_istances = 3

SapObject = []
SapModel = []

for jj in range(Num_SAP2000_istances):
    AttachToInstance = False
    SpecifyPath = False
    APIPath = 'C:\CSiAPIexample'
    Main_File_name = 'modelFixed'
    File_ext = '.sdb'
    ModelPath = APIPath + os.sep + 'IWSS23_TEST_parallel' + os.sep  #'API_1-001.sdb'
    helper = comtypes.client.CreateObject('SAP2000v1.Helper')
    helper = helper.QueryInterface(comtypes.gen.SAP2000v1.cHelper)
    if AttachToInstance:
        #attach to a running instance of SAP2000
        try:
            #get the active SapObject
                mySapObject = helper.GetObject("CSI.SAP2000.API.SapObject") 

        except (OSError, comtypes.COMError):
            print("No running instance of the program found or failed to attach.")
            sys.exit(-1)
    else:
        if SpecifyPath:
            try:
                #'create an instance of the SAPObject from the specified path
                mySapObject = helper.CreateObject(ProgramPath)

            except (OSError, comtypes.COMError):
                print("Cannot start a new instance of the program from " + ProgramPath)
                sys.exit(-1)
        else:
            try:
                #create an instance of the SAPObject from the latest installed SAP2000
                mySapObject = helper.CreateObjectProgID("CSI.SAP2000.API.SapObject")

            except (OSError, comtypes.COMError):
                print("Cannot start a new instance of the program.")
                sys.exit(-1)
        #start SAP2000 application
        mySapObject.ApplicationStart(Units=6)
        SapObject.append(mySapObject)
        SapModel.append(mySapObject.SapModel)
        ret = mySapObject.Hide()

# ret = mySapObject.Hide()
# ret = mySapObject.ApplicationStart(Units=6, FileName=ModelPath + Main_File_name + File_ext) #model.sdb
# SapModel = mySapObject.SapModel

LIST_HE = np.load('LIST_HE.npy')
LIST_IPE = np.load('LIST_IPE.npy')

problem = MyProblem(SapModel, SapObject, LIST_HE, LIST_IPE, ModelPath , Main_File_name , File_ext)

method = GA(pop_size=48,
            sampling=IntegerRandomSampling(),
            crossover=SBX(prob=1.0, eta=3.0, vtype=float, repair=RoundingRepair()),
            mutation=PM(prob=1.0, eta=3.0, vtype=float, repair=RoundingRepair()),
            eliminate_duplicates=True,
            )

res = minimize(problem,
               method,
               termination=('n_gen', 50),
            #    seed=1,
               save_history=True
               )

print("Best solution found: %s" % res.X)
print("Function value: %s" % res.F)
print("Constraint violation: %s" % res.CV)

n_evals = np.array([e.evaluator.n_eval for e in res.history])
opt = np.array([e.opt[0].F for e in res.history])

# plt.title("Convergence")
plt.plot(n_evals, opt, "--", color='C1')
plt.yscale("log")
plt.xlabel('OF evaluations [-]')
plt.ylabel('OF: Dead Load [kN]')
plt.tight_layout()
# plt.show()
plt.savefig('convergence.pdf')
# plt.close()

with open('n_evals.npy', 'wb') as f:
    np.save(f, n_evals)
with open('opt.npy', 'wb') as f:
    np.save(f, opt)
with open('resX.npy', 'wb') as f:
    np.save(f, res.X)
with open('resF.npy', 'wb') as f:
    np.save(f, res.F)


_X = np.row_stack([a.pop.get("X") for a in res.history])
feasible = np.row_stack([a.pop.get("feasible") for a in res.history])[:, 0]

with open('_X.npy', 'wb') as f:
    np.save(f, _X)
with open('feasible.npy', 'wb') as f:
    np.save(f, feasible)

print('finito')

for ii in range(Num_SAP2000_istances):
    ret = SapObject[ii].ApplicationExit(False)
    SapModel[ii] = None
    SapObject[ii] = None



