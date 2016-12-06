# -*- coding: utf-8 -*-
"""
Created on Tue Oct 11 14:12:21 2016

@author: elveleg
"""


class Shop:
    def __init__(self, name, category, directions):
        self.name = name
        self.category = category
        self.directions = directions

    def getName(self):
        return self.name

    def getCategory(self):
        return self.category

    def getDirections(self):
        return self.directions
