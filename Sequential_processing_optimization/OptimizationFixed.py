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


class MyProblem(Problem):

    def __init__(self,SapModel, LIST_HE, LIST_IPE, ModelPath):

        self.SapModel = SapModel
        self.LIST_HE = LIST_HE
        self.LIST_IPE = LIST_IPE
        self.ModelPath = ModelPath
        self.gen = 0

        n_var = 2

        xl = np.zeros((n_var,))
        xu = np.array([len(LIST_HE)-1,len(LIST_IPE)-1])

        super().__init__(n_var=2, n_obj=1, n_ieq_constr=2, xl=xl, xu=xu, vtype=int)

    def _evaluate(self, x, out, *args, **kwargs):

        out["F"] = np.zeros(x.shape[0])
        out["G"] = np.zeros((x.shape[0],2))

        

        for ii in range(x.shape[0]):

            start = time.time()
            ret = self.SapModel.SetModelIsLocked(False)

            for jj in range(1,16): 
                if jj<10:
                    ret = self.SapModel.FrameObj.SetSection(f"{jj}", self.LIST_HE[x[ii,0]])
                    # print(ret, self.LIST_HE[x[ii,0]])
                else:
                    ret = self.SapModel.FrameObj.SetSection(f"{jj}", self.LIST_IPE[x[ii,1]])
                    # print(ret, self.LIST_IPE[x[ii,0]])
            
            ret = self.SapModel.View.RefreshView(0, False)

            ret = self.SapModel.File.Save(self.ModelPath)
            ret = self.SapModel.File.OpenFile(self.ModelPath)

            ret = self.SapModel.Analyze.RunAnalysis()

            ret = self.SapModel.Results.Setup.SetCaseSelectedForOutput("DEAD")
            [NumberResults, LoadCase, StepType, StepNum, Fx, Fy, Fz, Mx, My, Mz, gx, gy, gz, ret] = self.SapModel.Results.BaseReact()


            out["F"][ii] = Fz[0]


            ret = self.SapModel.DesignSteel.StartDesign()

            g_summary = np.zeros(16)
            g_NMM = np.zeros(16)
            # g_V = np.zeros(16)
            for jj in range(1,16): 
                [NumberItems, FrameName, Ratio, RatioType, Location, ComboName, ErrorSummary, WarningSummary, ret] = self.SapModel.DesignSteel.GetSummaryResults(f'{jj}')
                g_summary[jj-1] = Ratio[0]
                [NumberItems, FrameName, Value, ret] = self.SapModel.DesignSteel.GetDetailResultsValue(f"{jj}", 0, 2, "TotalRatio")
                g_NMM[jj-1] = Value[0]
                # [NumberItems, FrameName, Value, ret] = self.SapModel.DesignSteel.GetDetailResultsValue(f"{jj}", 0, 3, "V2Ratio")
                # g_V[jj-1] = Value[0]

            out["G"][ii,0] = sum(np.max((g_NMM - 1, np.zeros(16)), axis=0))
            out["G"][ii,1] = sum(np.max((g_summary - 1, np.zeros(16)), axis=0))

            end = time.time()
            print('Elapsed time :',end - start,'s')
        
        self.gen += 1
        print(f'gen {self.gen}\n','F=', out["F"], '\n', 'G=', out["G"], '\n\n')















AttachToInstance = True
SpecifyPath = False
APIPath = 'C:\CSiAPIexample'
ModelPath = APIPath + os.sep + 'IWSS23_TEST_1' + os.sep + 'modelFixed.sdb' #'API_1-001.sdb'
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
    mySapObject.ApplicationStart()

ret = mySapObject.Hide()
ret = mySapObject.ApplicationStart(Units=6, FileName=ModelPath) #model.sdb
SapModel = mySapObject.SapModel

LIST_HE = np.load('LIST_HE.npy')
LIST_IPE = np.load('LIST_IPE.npy')

problem = MyProblem(SapModel, LIST_HE, LIST_IPE, ModelPath)

method = GA(pop_size=50,
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

#plt.title("Convergence")
plt.plot(n_evals, opt, "--", color='C1')
plt.yscale("log")
plt.xlabel('OF evaluations [-]')
plt.ylabel('OF: Dead Load [kN]')
plt.tight_layout()
# plt.show()
plt.savefig('convergence.pdf')
plt.close()

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

def plot(*args, show=True, labels=None, no_fill=False, kwargs={}):
    F = args[0]

    if F.ndim == 1:
        print("Cannot plot a one dimensional array.")
        return

    n_dim = F.shape[1]

    if n_dim == 2:
        ret = plot_2d(*args, labels=labels, no_fill=no_fill, kwargs=kwargs)
    elif n_dim == 3:
        ret = plot_3d(*args, labels=labels, no_fill=no_fill, **kwargs)
    else:
        print("Cannot plot a %s dimensional array." % n_dim)
        return

    if labels:
        plt.legend()

    if show:
        plt.show()

    return ret


def plot_2d(*args, labels=None, no_fill=False, kwargs={}):
    if no_fill:
        kwargs = dict(
            s=20,
            facecolors='none',
            edgecolors='r'
        )
    elif bool(kwargs):
        kwargs = {'s':10}

    for i, F in enumerate(args):
        if labels:
            plt.scatter(F[:, 0], F[:, 1], label=labels[i], **kwargs)
        else:
            plt.scatter(F[:, 0], F[:, 1], **kwargs)


plot(_X[feasible], _X[np.logical_not(feasible)], res.X[None,:] ,
              labels=["Feasible", "Infeasible", "Best"] , kwargs={'s':5}
              )

ax = plt.gca()
ax.annotate('Best Solution', xy=tuple(res.X), xycoords='data',
            xytext=(0.9, 0.45), textcoords='axes fraction',
            ha='right',
            arrowprops=dict( facecolor='black', shrink=0.025, width=1 , headwidth=8 , headlength = 8 )
            )
plt.scatter(res.X[0],res.X[1],facecolor='white', edgecolors='C2', s=5, lw=0.1)
plt.xlabel('Columns HE profile index [-]')
plt.ylabel('Beams IPE profile index [-]')
plt.savefig('exploration.pdf')


print('fine')





