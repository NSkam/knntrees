# knntrees
Representing collection docs as three dimensional points, constructing different trees (Octree, Kd tree and R-tree) and then executing k-NN queries on them.


Steps of the algorithm:

Step #1:
  The documents and the queries of the collection are analyzed into document and query vectors using the apriori algorithm and the Set-based model based on n keyowrds.
  
Step #2:
  Using the principal component analysis (PCA), the dimensions of the vectors are reduced to 3.
  
Step #3:
  The document vectors are then used to construct tree structures, namely Octrees, Kd-trees and R-trees.
  
Step #4:
  Using the query vectors, the algorithm executes k-NN queries on the aforementioned trees.
  
Inputs:
  The documents of the collection are in seperate files on the disk (in a folder called txtfiles), while the queries are in the form of an array, in which every query corresponds to one element of the array.
  
Outputs:
  One array with the k nearest points to the query point, and one array with the euclidean distance of each of the k points.
