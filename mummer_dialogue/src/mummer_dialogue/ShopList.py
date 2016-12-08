# -*- coding: utf-8 -*-
"""
Created on Tue Oct 11 13:49:48 2016

@author: elveleg
"""

import numpy as np
from Shop import Shop
import rospy


class ShopList(list):
    def __init__(self, path='', empty=False):
        if not empty:
            data = np.recfromcsv(path, delimiter=';')
            rospy.loginfo(data)
            for s in data:
                shop = Shop(s[0], s[1], s[2], s[4])
                self.append(shop)
                print shop.getName()

    def filteredCategory(self, cat):
        new = ShopList(empty=True)
        for s in self:
            if s.getCategory() == cat:
                new.append(s)
        return new

    #    def filteredCategory(self, cat):
    #        for s in self:
    #            if s.getCategory() != cat:
    #                self.remove(s)
    #        return self

    def getShop(self, shopName):
        for s in self:
            if s.getName() == shopName:
                return s

    def enumShops(self):
        res = ""
        for i in self[:-1]:
            res += " " + i.getName() + ", "

        for i in self[-1:]:
            res += " and " + i.getName()
        return res

    def getDirections(self, shopName):
        return self.getShop(shopName).getDirections()
                
    def getId(self, shopName):
        return self.getShop(shopName).getId()
