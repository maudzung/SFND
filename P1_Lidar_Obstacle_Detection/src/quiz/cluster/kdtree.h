/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	void insert_helper(Node ** node, uint depth, std::vector<float> point, int id) {
		if (*node ==NULL) {
			*node = new Node(point, id);
		}
		else {
			uint cd = depth % 2;
			if (point[cd] < (*node)->point[cd]) {
				insert_helper(&(*node)->left, depth + 1, point, id);
			}
			else {
				insert_helper(&(*node)->right, depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_helper(&root, 0, point, id);
	}

	void search_helper(std::vector<int>& ids, const Node *node, std::vector<float> target, float distanceTol, int depth) {
		if (node != NULL) {
			float diff_x = node->point[0] - target[0];
			float diff_y = node->point[1] - target[1];
			if ((diff_x >= - distanceTol) &&
				(diff_x <= distanceTol) && 
				(diff_y >= - distanceTol) &&
				(diff_y <= distanceTol)) {
				float dist = std::sqrt(diff_x * diff_x + diff_y * diff_y);
				if (dist <= distanceTol) {
					ids.push_back(node->id);
				}
			}
			uint cd= depth % 2;
			if (target[cd] - distanceTol < node->point[cd]) {
				search_helper(ids, node->left, target, distanceTol, depth + 1);
			}
			if (target[cd] + distanceTol > node->point[cd]) {
				search_helper(ids, node->right, target, distanceTol, depth + 1);
			}
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(ids, root, target, distanceTol, 0);
		return ids;
	}
	

};




