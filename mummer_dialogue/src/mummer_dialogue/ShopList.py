# -*- coding: utf-8 -*-
"""
Created on Tue Oct 11 13:49:48 2016

@author: elveleg
"""

from Shop import Shop
from pprint import pprint
import unicodedata


class ShopList(list):
    def __init__(self, db=None, empty=False):
        if not empty:
            for entry in db:
                shop = Shop(
                    id=int(entry['shop_id']), 
                    name=entry['shopName'].lower(), 
                    category=entry['category'].lower(),
                    directions=entry['directions'], 
                    sales=self.str2bool(entry['sales']),
                    name_synonyms=entry["shopNameSynonyms"],
                    category_synonyms=entry["categorySynonyms"],
                    sold_items=entry["soldItems"]
                )
                self.append(shop)
                print shop.getName(),':',shop.getNames()
                pprint(entry)

    def filteredCategory(self, cat):
        new = ShopList(empty=True)
        for s in self:
            if s.getCategory() == cat:
                new.append(s)
        return new
    
    def robust_decode(self, bs):
        '''Takes a byte string as param and convert it into a unicode one.
    First tries UTF8, and fallback to Latin1 if it fails'''
        cr = None
        try:
            cr = bs.decode('utf8')
        except UnicodeDecodeError:
            cr = bs.decode('latin1')
        return cr
        
    #    def filteredCategory(self, cat):
    #        for s in self:
    #            if s.getCategory() != cat:
    #                self.remove(s)
    #        return self

    def getShop(self, shopName):
        shopName = self.robust_decode(shopName)
        for s in self:
            if shopName.lower() in s.getNames():
                return s

    def enumShops(self):
        if len(self) == 0: return "none"        
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
    
    def getUniqueCategories(self):
        ret = []        
        for s in self:
            if not s.getCategory() in ret:
                ret.append(s.getCategory())
        return ret
    
    def str2bool(self, s):
        s = s.lower()
        if s == 'true':
            return True
        elif s == 'false':
            return False
        else:
            raise ValueError("Cannot convert {} to a bool".format(s))
