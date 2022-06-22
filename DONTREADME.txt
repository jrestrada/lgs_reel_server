Talking Points:

The order matters, so a flexible action server would take in a pattern order.
and have the flexibility to do an action either continuously or once 


A minimum crawl forward/backward cycle command would have six elements


TO FIX: What happens when you are executing a goal and you get a new one??
TO FIX: figure out count logic. what subtracts??? if extender extends????

What is going to let me keep track of how many counts forward???




python stuff:

How to extract elements from goal:::::
        self._crawl_counter = goal_handle.request.crawlercommand.crawlpattern[1].item() #use item method to extract int from numpy.int