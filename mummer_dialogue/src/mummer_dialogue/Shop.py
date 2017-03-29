# -*- coding: utf-8 -*-
"""
Created on Tue Oct 11 14:12:21 2016

@author: elveleg
"""

import json
import rospy


class Shop:
    def __init__(self, id, name, name_synonyms, category, category_synonyms, directions, sales, sold_items):
        self.id = id
        self.name = name.lower()
        try:
            self.name_synonyms = [x.lower() for x in json.loads(name_synonyms)]
        except ValueError as e:
            rospy.logdebug(e)
            self.names = [self.name]
        else:
            self.names = self.name_synonyms
            self.names.append(self.name)
        self.category = category.lower()
        try:
            self.category_synonyms = [x.lower() for x in json.loads(category_synonyms)]
        except ValueError as e:
            rospy.logdebug(e)
            self.categories = [self.category]
        else:
            self.categories = self.category_synonyms
            self.categories.append(self.category)
        self.directions = directions
        self.sales = sales
        try:
            self.sold_items = [x.lower() for x in json.loads(sold_items)]
        except ValueError as e:
            rospy.logdebug(e)
            self.sold_items = []

    def getId(self):
        return self.id

    def getName(self):
        return self.name
        
    def getNameSynonyms(self):
        return self.name_synonyms
        
    def getNames(self):
        return self.names

    def getCategory(self):
        return self.category
        
    def getCategorySynonyms(self):
        return self.category_synonyms
        
    def getCategories(self):
        return self.categories

    def getDirections(self):
        return self.directions

    def getSales(self):
        return self.sales
        
    def getSoldItems(self):
        return self.sold_items
