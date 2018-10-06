#include <iostream>
using namespace std;
template <typename elemType>
class BinaryTree;

template <typename valType>
class BTnode
{
	public:
		BTnode(const valType &val);
		
		void insert_value(const valType &val);
		void lchild_leaf(BTnode* leaf, BTnode* subtree);
		void remove_value(const valType &val, BTnode* &prev);
		friend class BinaryTree<valType>;
	private:
		valType  _val;
		int      _cnt;
		BTnode*  _lchild;
		BTnode*  _rchild;		
 }; 
 
template <typename elemType>
class BinaryTree
 {
 	public:
 		BinaryTree();
 		BinaryTree(const BinaryTree&);
 		~BinaryTree();
 		BinaryTree& operator=(const BinaryTree&);
 		
 		bool empty();
 		void clear()
		{    
			if (_root)
		   {
		 		clear(_root);  _root = 0;
		   }
		}
		void insert(const elemType &elem); 
		void remove(const elemType &elem);
		void display_value(BTnode<elemType>* pt, ostream &os);
		void preorder(BTnode<elemType>* pt, ostream &os) const;
		void inorder(BTnode<elemType>* pt, ostream &os) const;
		void postorder(BTnode<elemType>* pt, ostream &os) const;
		void remove_root();
 	private:
 		BTnode<elemType>*  _root;
 		void copy(BTnode<elemType>* tar, BTnode<elemType>* src);
 		void clear(BTnode<elemType>*);
 };
