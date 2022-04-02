from getQueries import *
from getRelevant import *
from AnalyzeCollectionDocs import *
import nltk
from nltk.corpus import stopwords
import string
from sklearn.decomposition import PCA

def DocumentReprsentationAsPoints():
    
    #Analyze the collection
    postingList, collectionTerms, docsLen, fileList = AnalyzeCollection()
    #print(postingList)
    #print(fileList)
    print("Done Analizing the documents")

    #Get and preprocess the list of Stopwords
    stop_words = set(stopwords.words('english'))
    table = str.maketrans('', '', string.punctuation)
    stop_words = [w.translate(table) for w in stop_words]
    stop_words = [word.upper() for word in stop_words]

    
        
    #Preprocess the Query and get the keywords
    Q = "CF Fibrosis Cystic Patients Effects Properties"
    print('Keywords ======== ',Q)
    Q = Q.upper()
    Q = Q.split()
    keywords = [w for w in Q if not w in stop_words]
    keywords = list(set(Q))  # to ignore duplicate words as queries
    #print(keywords)
    numOfTermsets = (2 ** (len(keywords))) - 1
        
    #Generate the Termesets of the Keywords using apriori
    #print(len(collectionTerms))
    One_termsets = one_termsets(keywords, collectionTerms, postingList, 0)
    #print(One_termsets)
    
    final_list = apriori(One_termsets,0)
    #print(final_list)
 
    #Calculate the termset frequency for every doc
    docs, doc_vectors = fij_calculation(fileList, final_list, postingList, collectionTerms)
    print(len(doc_vectors))

    #Calculate the inverse document frequency of every termset
    idf_vec = calculate_idf(final_list, len(fileList))

    #Calculate the tf*idf for every document
    documentmatrix = doc_rep(doc_vectors, idf_vec)
    #print(len(documentmatrix))
 
    #Transform the document matrix into a numpy array and run pca to reduce dimensions to 3 using pca
    documentMatrix = numpy.array(documentmatrix)
    #print(documentMatrix)
    #print(len(documentMatrix[1]))
   
    pca = PCA(n_components=3)
    transformedDocumentMatrix = pca.fit_transform(documentMatrix, y=None)
    #print(transformedDocumentMatrix)

    return transformedDocumentMatrix