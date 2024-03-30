#pragma once
#include"main.h"

class BST
{
public:
	BST(int key, int value);
	void testBST();
	//创建树
	BST* inster(BST* root, int key, int value);
	//递归搜索
	BST* search_recursive(BST* root, int key);
	//迭代搜索
	BST* search_iterative(BST* root, int key);
	//中序遍历
	void inorder(BST* root);
	//先序遍历
	void preorder(BST* root);
	//后序遍历
	void postorder(BST* root);
	void output(BST* root);
private:
	BST *left;
	BST *right;
	int key;
	int value;

};

