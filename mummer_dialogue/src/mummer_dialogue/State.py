# -*- coding: utf-8 -*-
"""
Created on Thu Oct 20 14:34:57 2016

"""


class state:
    def __init__(self, tskC, tskF, prevA, dist, ctx, usrEng, mode, timeout, usrTerm, bye, usrEngC, lowConf, turn):
        self.tskCompleted = tskC
        self.tskFilled = tskF
        self.prevAct = prevA
        self.distance = dist
        self.ctxTask = ctx
        self.turnTaking = turn
        self.usrEngaged = usrEng
        self.mode = mode
        self.timeout = timeout
        self.usrTermination = usrTerm
        self.bye = bye
        self.usrEngChat = usrEngC
        self.lowConf = lowConf

    def printState(self):
        print "State: ", self.tskCompleted, self.tskFilled, self.prevAct, self.distance, self.ctxTask, self.usrEngaged, self.mode, self.timeout, self.usrTermination, self.bye, self.usrEngChat, self.lowConf, self.turnTaking

    def getTurn(self):
        return self.turnTaking
