from threading import Lock

# From Lectures
class Node:
    def __init__(self, value):
        self.value = value
        self.next = None

class Queue:
    def __init__(self):
        self.head = self.tail = Node(-1)
        self.headLock = Lock()
        self.tailLock = Lock()

    def push(self, value):
        new = Node(value)
        with self.tailLock:
            self.tail.next = new
            self.tail = new
        
    
    def pop(self):
        with self.headLock:
            next = self.head.next
            if next is None:
                return "Empty"
            self.head = next
            value = next.value
        return value