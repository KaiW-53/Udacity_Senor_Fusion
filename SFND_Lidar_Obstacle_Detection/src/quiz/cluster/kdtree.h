/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
    
	void insertHelper(Node** node, int depth, std::vector<float> point, int id) {
		if (*node == NULL) {
			*node = new Node(point, id);
			return;
		}
		
		int curr_dim = depth % 3;
		if (point[curr_dim] < (*node)->point[curr_dim]) {
			insertHelper(&((*node)->left), depth+1, point, id);
		}
		else {
			insertHelper(&((*node)->right), depth+1, point, id);
		}
		return;
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}


	void searchHelper(Node** node, int depth, std::vector<float> target, std::vector<int> &ids, float tol) {
		if (*node == NULL) {
			return;
		}
		int curr_dim = depth % 3;
		std::vector<float> point = (*node)->point;
		float d = point[curr_dim] - target[curr_dim];
		float dist = sqrt((point[0]-target[0])*(point[0]-target[0]) + (point[1]-target[1])*(point[1]-target[1])
		                  + (point[2]-target[2])*(point[2]-target[2]));
		if (dist < tol) ids.push_back((*node)->id);
		if (d > 0) {
			searchHelper(&((*node)->left), depth+1, target, ids, tol);
			if (d < tol) {
				searchHelper(&((*node)->right), depth+1, target, ids, tol);
			}
			return;
		}
		d = abs(d);
		searchHelper(&((*node)->right), depth+1, target, ids, tol);
		if (d < tol) {
			searchHelper(&((*node)->left), depth+1, target, ids, tol);
		}
		return;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(&root, 0, target, ids, distanceTol);
		return ids;
	}
	

};




