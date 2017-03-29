# -*- coding: utf-8 -*-

import yaml
from pprint import pprint


class ConceptGenerator(object):
    def __init__(self, static_concepts_path):
        self.__concepts_dict = self.__load_yaml_file(static_concepts_path)["concepts"]
        
    def __load_yaml_file(self, file_path):
        with open(file_path, 'r') as f:
            return yaml.load(f)
        
    def generate(self, shop_list):
        shops = []        
        for s in shop_list:
            shops.extend(s.getNames())
        self.__concepts_dict["shop"] = [x for x in self.qichat_hack(set(shops))]
        for c in shop_list.getUniqueCategories():
            print c
            f = shop_list.filteredCategory(c)
            categories = []            
            for s in f:
                print "Categories:",s.getCategories()
                categories.extend(s.getCategories())
                categories.extend(s.getSoldItems())
            self.__concepts_dict[c] = [x for x in self.qichat_hack(set(categories))]
        concepts = '\n'.join(['concept:(%s) [%s]' % (k,' '.join(v)) for k,v in self.__concepts_dict.items()])
        concepts += '\n'
        pprint(self.__concepts_dict)
        return concepts
        
    def qichat_hack(self, l):
        for e in l:
            if ' ' in e:
                yield '"'+e+'"'
            else:
                yield e
