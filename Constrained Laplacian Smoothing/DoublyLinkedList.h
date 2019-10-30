#ifndef DOUBLY_LINKED_LIST
#define DOUBLY_LINKED_LIST 

#include <cstddef>
#include "NeighborhoodVertex.h"

struct Node {
	NeighborhoodVertex* data;
	Node* next;
	Node* prev;
};

class DoublyLinkedList {
public:

	DoublyLinkedList();

	void insert(NeighborhoodVertex* value);
	Node* find(NeighborhoodVertex* value);

	unsigned int getLength();

private:

	Node* head;
	unsigned int length;

};

#endif
