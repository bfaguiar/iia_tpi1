#  Name: Bruno Filipe Oliveira Aguiar

#N.MEC: 80177

from tree_search import *

import functools

from scipy import optimize 

class MyNode(SearchNode):
    def __init__(self, state,parent, depth, cost, evalfunc,   children):
        super().__init__(state, parent)
        self.depth = depth
        self.cost = cost
        self.evalfunc = evalfunc
        self.children = children

    def have_parent(self, state): #evita repetir os nós
        if self.parent == None:
            return False
        return self.parent.state == state or self.parent.have_parent(state)

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',max_nodes=None): 
        super().__init__(problem, strategy)
        self.root = MyNode(problem.initial, None, 0, 0, self.problem.domain.heuristic(problem.initial, problem.goal), None) #, , 
        self.open_nodes = [self.root]
        self.solution_cost = 0
        self.solution_length = 0
        self.total_nodes = 1 # porque ja esta no open nodes. o root.
        self.max_nodes = max_nodes
        self.terminal_nodes = 1
        self.non_terminal_nodes = 0
        self.ramification = 0

    def astar_add_to_open(self,lnewnodes):
        self.open_nodes = sorted(self.open_nodes + lnewnodes, key=lambda node: node.evalfunc + node.cost)
  
    def effective_branching_factor(self): # solucao analitica uasndo o método de Newton-Raphson , com o N(1/d) como ebf.
        ebf = self.total_nodes ** (1/self.solution_length)
        return optimize.newton(lambda b: b/(b-1)*(b**self.solution_length - 1)- self.total_nodes, ebf)
    
    def update_ancestors(self,node):
            if len(node.children): 
                node.evalfunc = sorted(node.children, key = lambda x: x.evalfunc)[0].evalfunc # assign da children com menor evalfunc ao node.evalfunct
            if node.parent == None:
                return node.evalfunc
            return node.evalfunc and self.update_ancestors(node.parent) 

    def discard_worse(self):
        lista_n_t = [node for node in self.open_nodes if node.children != None] # lista de nos nao terminais
        node_with_leafs = [node for node in lista_n_t for node_child in node.children if node_child.children == None ] # lista de nos nao terminais que tem children que sao folhas
        if node_with_leafs != []:
            upper = functools.reduce(lambda x, y: x if x.evalfunc > y.evalfunc else y, node_with_leafs)
            for c in upper.children:
                if c in self.open_nodes():
                    self.open_nodes().remove(c)
    
    def search2(self):
        while self.open_nodes != []:
            node = self.open_nodes.pop(0)
            node.children = []
            self.solution_cost =  node.cost
            if self.problem.goal_test(node.state):
                self.solution_length = node.depth
                return self.get_path(node)
            lnewnodes = []
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state,a)
                if not node.have_parent(newstate):
                    newnode = MyNode(newstate,node, node.depth + 1, node.cost + self.problem.domain.cost(node.state, a), self.problem.domain.heuristic(newstate, self.problem.goal) + node.cost + self.problem.domain.cost(node.state, a) , None)
                    lnewnodes.append(newnode)
                    node.children.append(newnode)
                    self.total_nodes = self.total_nodes + 1
            if(self.max_nodes != None and self.total_nodes > self.max_nodes):
                self.discard_worse()
            if len(lnewnodes):
                self.terminal_nodes += len(lnewnodes)
            else: self.non_terminal_nodes += 1
            self.terminal_nodes -= 1
            self.add_to_open(lnewnodes)
            self.update_ancestors(node)
        return None

    # shows the search tree in the form of a listing
    def show(self,heuristic=False,node=None,indent=''):
        if node==None:
            self.show(heuristic,self.root)
            print('\n')
        else:
            line = indent+node.state
            if heuristic:
                line += (' [' + str(node.evalfunc) + ']')
            print(line)
            if node.children==None:
                return
            for n in node.children:
                self.show(heuristic,n,indent+'  ')


