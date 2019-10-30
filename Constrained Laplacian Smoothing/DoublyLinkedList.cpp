#include "DoublyLinkedList.h"

DoublyLinkedList::DoublyLinkedList() {
	length = 0;
	head = NULL;
}

void DoublyLinkedList::insert(NeighborhoodVertex* value) {

	Node* node = new Node();
	node->data = value;

	if (head == NULL) {
		head = node;
		head->next = head;
		head->prev = head;
	} else {
		head->prev->next = node;
		node->prev = head->prev;
		head->prev = node;
		node->next = head;
	}

	length++;
}

Node* DoublyLinkedList::find(NeighborhoodVertex* value) {

	if (length == 0) {
		return NULL;
	}

	Node* temp = head;
	do {

		if (temp->data == value) {
			return temp;
		}

		temp = temp->next;

	} while (temp != head);

	return NULL;
}

unsigned int DoublyLinkedList::getLength() {
	return length;
}
