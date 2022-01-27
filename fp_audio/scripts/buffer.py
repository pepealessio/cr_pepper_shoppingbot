class Buffer(object):
    """Adapter of the list who mantains just the last max_size elements."""
    
    def __init__(self, max_size):
        """Init an empty buffer."""
        self._list = list() 
        self._max_size = max_size

    def put(self, x):
        """Add x in the buffer but if the buffer is full remove the oldest element.

        Args:
            x (Any): The element to insert in the buffer.
        """
        if len(self._list) == self._max_size:
            self._list.pop(0)
        self._list.append(x)
    
    def pop(self):
        """Return the oldest element in the buffer.

        Returns:
            Any: The oldest element in the buffer.
        """
        return self._list.pop(0)

    def isEmpty(self):
        """Return true if the buffer is empty.
        """
        return len(self._list) == 0
    
    def clear(self):
        """Empty the buffer.
        """
        self._list.clear()
