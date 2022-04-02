import os
from operator import itemgetter
import numpy
from math import log
from getQueries import *

def AnalyzeCollection():
    file_list = []
    for item in os.listdir('txtfiles'):
        name = os.path.join("txtfiles", item)
        if os.path.getsize(name) == 0:
            print('%s is empty:' % name)
            os.remove(name)
        else:
            file_list.append([name, os.path.getsize(name)])
    file_list = sorted(file_list, key=itemgetter(1), reverse=True)
    
    postinglist =[]
    collection_terms = []
    doc_length =[]
    
    for file in file_list:
        filename = file[0]
        #print(file)
        with open(filename, 'r') as fd:
        #list containing every word in text document
            text = fd.read().split()
            temp = getPostingList(filename,text,postinglist,collection_terms)
            postinglist = temp[0]
            collection_terms = temp[1]
            doc_length.append(temp[2])
            #print('Analyzed File:' + filename)
    queries = getQueries()
    counter = 1210
    for query in queries:
        qname = "txtfiles\\" + str(counter)
        file_list.append([qname, 15])
        text = query.split()
        text = [word.upper() for word in text]
        temp = getPostingList(qname,text,postinglist,collection_terms)
        postinglist = temp[0]
        collection_terms = temp[1]
        doc_length.append(temp[2])
        counter+=1
        
    return postinglist, collection_terms, doc_length, file_list     

def getPostingList(name,text, postingl,collection_terms):

    for term in text:
        if term not in postingl:
            collection_terms.append(term)
            postingl.append(term)
            postingl.append([name, text.count(term)])
        else:
            existingtermindex = postingl.index(term)
            if name not in postingl[existingtermindex + 1]:
                postingl[existingtermindex + 1].extend([name, text.count(term)])
    return postingl, collection_terms, len(text)


def one_termsets(keywords, termns, postinglist, minfreq):
    One_termsets = []
    for word in keywords:
        if word in termns:
            i = postinglist.index(word)
            doc = postinglist[(i + 1)]
            doc = doc[::2]
            word = [''.join(word)]
            #print(doc)
            if len(doc) > minfreq:
                One_termsets.append([word, doc])
        else:
            print('word %s has not required support or it already exists:' % word)
    return One_termsets

def apriori(l1, minfreq):
    final_list = []
    final_list.append(l1)
    k = 2
    l = l1
    print('=========Generating frequent sets ============')
    while (l != []):
        c = apriori_gen(l)
        l = apriori_prune(c, minfreq)
        final_list.append(l)
        k += 1
    return final_list


def apriori_gen(itemset):
    candidate = []
    length = len(itemset)
    for i in range(length):
        ele = itemset[i][0]
        for j in range(i + 1, length):
            ele1 = itemset[j][0]
            if ele[0:len(ele) - 1] == ele1[0:len(ele1) - 1]:
                c=[]
                for k in ele + ele1:
                    if k not in c:
                        c.append(k)
                candidate.append([c, list(set(itemset[i][1]) & set(itemset[j][1]))])
    return candidate

def apriori_prune(termsets_list, min_support):
    prunedlist = []
    for j in termsets_list:
        if len(j[1]) > min_support:
            prunedlist.append([j[0], j[1]])

    return prunedlist
        
    
def fij_calculation(file_list, final_list, plist, trms):
    
    docs =[]
    doc_list = []
    weight_doc_matrix = []
    doc_vec = []
    
    #print(file_list)
    #print(final_list)
    #print("\n\n\n\n")
    #print(plist)
    #sprint(trms)
    for itemsetList in final_list:
        #print("i ======= %s"%i)
        #print(itemsetList)
        for itemset in itemsetList:
            
            #print(file[0])
            for file in file_list:
                #print(file[0])
                docs.append(file[0])
                
                #print(itemset[1])
                if file[0] in itemset[1]:
                    #print(plist[1])
                    itemsetTerms = itemset[0]
                    for itemset_term in itemsetTerms:
                        for j in range(1,len(plist),2):
                            #print(plist[j])
                            #print(plist[j-1])
                            if itemset_term in plist[j-1]:
                                if file[0] in plist[j]:
                                    file_index = plist[j].index(file[0])
                                    weight =  plist[j][file_index+1]
                                    #print(file_index)
                                    weight_doc_matrix.append(weight)
                                    #print(file[0])
                                    #print(weight)
                                    #print(weight_doc_matrix)
                    doc_vec.append(min(weight_doc_matrix))
                    weight_doc_matrix = []
                else:
                    doc_vec.append(0)
            doc_list.append(doc_vec)
            doc_vec = []
        #print(doc_list)
    doc_list = numpy.transpose(doc_list)
    #print(doc_list)
    doc_list = doc_list.tolist()
    #print(doc_list)
    return docs, doc_list
    
    
def calculate_idf(termsetsL, numofdocs):
    print('=====calculating idfs============')
    idf_vector = []
    for ts in termsetsL:  # iterate based on the number of terms in termset
        for item in ts:  # iterate all termsets with the same number of terms in set
            Nt = len(item[1])
            N = numofdocs
            if Nt != 0:
                idf = log(1 + (N / Nt))
                idf_vector.append(idf)
            else:
                idf_vector.append(0)
    return idf_vector

def doc_rep(doc_vec, idf_vec):
    test = numpy.zeros((len(doc_vec), len(idf_vec)))
    for i in range(len(doc_vec)):
        # print(docs[i])
        for j in range(len(idf_vec)):
            # print(doc_vec[i][j])
            if doc_vec[i][j] > 0:
                test[i][j] = (1 + log(doc_vec[i][j])) * idf_vec[j]
            else:
                test[i][j] = 0
    return test