#!/usr/bin/env python3

# Basic Link list

class Node:
    def __init__(self, value):
        self.value = value
        self.next = None

class SlinkedList:
    def __init__(self):
        self.head = None

    def listprint(self):
        current = self.head
        while current:
            print(current.value, end=' ')
            current = current.next
        print()

list = SlinkedList()
list.head = Node(1)
l2 = Node(2)
l3 = Node(3)

#Link the nodes
list.head.next = l2

#Link the next node
l2.next = l3

list.listprint()  # Output: 1 2 3
