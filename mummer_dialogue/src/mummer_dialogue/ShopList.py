# -*- coding: utf-8 -*-
"""
Created on Tue Oct 11 13:49:48 2016

@author: elveleg
"""

from Shop import Shop


class ShopList(list):
    def __init__(self, db=None, empty=False):
        if not empty:
            for entry in db:
                shop = Shop(
                    int(entry['shop_id']), 
                    entry['shopName'].lower(), 
                    entry['category'].lower(),
                    entry['directions'], 
                    self.str2bool(entry['sales'])
                )
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

    #lowercase the shopname first
    def getShop(self, shopName):
        if shopName.lower() == "pcworld":
            shopName = "pc world"
        elif shopName.lower() in ("disco","school"):
            shopName = "tesco"
        elif shopName.lower() in ("coasta","coastal"):
            shopName = "costa"
        elif shopName.lower() in ("premark",):
            shopName = "primark"
        for s in self:
            if s.getName() == shopName.lower():
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

    def filteredSales(self):
        new = ShopList(empty=True)
        for s in self:
            if s.getSales():
                new.append(s)
        return new

    def getShops(self):
        new = []
        for s in self:
            new.append(s.getName())

        return new
    
    def str2bool(self, s):
        s = s.lower()
        if s == 'true':
            return True
        elif s == 'false':
            return False
        else:
            raise ValueError("Cannot convert {} to a bool".format(s))
