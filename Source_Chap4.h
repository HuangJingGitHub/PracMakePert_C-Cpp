#include <iostream>

template <typename Object>
struct TreeNode
{
	Object element;
	TreeNode *firstChild;
	TreeNode *nextSibling;
};

/*template <typename Object>
struct BinaryNode
{
	Object element;
	BinaryNode *left;
	BinaryNode *right;
};*/

/*Implementation of binary search tree*/
template <typename Comparable>
class BinarySearchTree
{
public:
	BinarySearchTree()
		: *root{ nullptr } {}
	BinarySearchTree(const BinarySearchTree &rhs)
	{
		BinarySearchTree copy = rhs;
		root = copy.root;
		copy.root = nullptr;           // ***** 
	}
	BinarySearchTree(BinarySearchTree &&rhs)
	{
		root = rhs.root;
		rhs.root = nullptr;
	}
	~BinarySearchTree()
	{
		root = nullptr;
	}

	const Comparable & findMin() const;
	const Comparable & findMax() const;
	bool contain(const Comparable & x) const
	{
		return contain(x, root);
	}
	void printTree(ostream & out = cout) const;

	void makeEppty();
	void insert(const Comparable & x)
	{
		insert(x, root);
	}
	void insert(const Comparable && x)
	{
		insert(x, root);
	}
	void remove(const Comparable & x)
	{
		remove(x, root);
	}

	BinaeySearchTree & operator=(const BinarySearchTree & rhs);
	BinarySearchTree & operator=(BinarySearchTree && rhs);

private:
	struct BinaryNode
	{
		Comparable element;
		BinaryNode	*left;
		BinaryNode	*right;

		BinaryNode(const Comparable & theElement, BinaryNode *lt, BinaryNode *rt)
			: element{ theElement }, left{ lt }, right{ rt } { }
		BinaryNode(Comparable && theElement, BinaryNode *lt, BinaryNode *rt)
			: element{ std::move(theElement) }, left{ lt }, right{ rt } { }
	};
	
	BinaryNode *root;

	void insert(const Comparable & x, BinaryNode * & t);
	void insert(Comparable && x, BinaryNode * & t);
	void remove(const Comparable & x, BinaryNode * & t);
	BinaryNode * findMin(BinaryNode *t) const;
	BinaryNode * findMax(BinaryNode *t) const;
	bool contain(const Comparable & x, BinaryNode *t) const;
	void makeEmpry(BinaryNode * & t);
	void printTree(BinaryNode *t, ostream & out) const;
	BinaryNode * clone(BinaryNode * t) const;
};