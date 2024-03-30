#include "BST.h"

BST::BST(int key, int value)
{
	this->left = nullptr;
	this->right = nullptr;
	this->key = key;
	this->value = value;
}

//创建树
BST* BST::inster(BST *root, int key, int value) {
	if (root == nullptr) {
		BST bst(key, value);
		root = &bst;
	}
	else
	{
		if (key < root->key)
			root->left = inster(root->left, key, value);
		else if (key > root->key)
			root->right = inster(root->right, key, value);
	}
	return root;
}

//递归搜索
BST* BST::search_recursive(BST* root, int key) {
	if (root == nullptr || root->key == key)
		return root;
	if(key < root->key)
		search_recursive(root->left, key);
	else if(key > root->key)
		search_recursive(root->right, key);
}

//迭代搜索
BST* BST::search_iterative(BST* root, int key) {
	BST* current_node = root;
	while (current_node != nullptr)
	{
		if (current_node->key == key)
			return current_node;
		else if(current_node->key < key)
			current_node = current_node->left;
		else if (current_node->key > key)
			current_node = current_node->right;
	}
}

//中序遍历
void BST::inorder(BST* root) {
	inorder(root->left);
	output(root);
	inorder(root->right);
}

//先序遍历
void BST::preorder(BST* root) {
	output(root);
	inorder(root->left);
	inorder(root->right);
}

//后序遍历
void BST::postorder(BST* root) {
	inorder(root->left);
	inorder(root->right);
	output(root);
}

void BST::output(BST* root) {
	std::cout << "key: " << root->key << " value: " << root->value << "\n" << std::endl;
}

void BST::testBST() {
	int N = 9; 
	BST *root = nullptr;
	for (int i = 0; i < N; i++)
	{
		root = inster(root, rand(), i+1);
	}
	inorder(root);
}
