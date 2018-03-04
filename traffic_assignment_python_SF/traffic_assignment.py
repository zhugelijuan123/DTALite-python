'''
Traffic Assignment in python.
'''
import math
import csv


_MAX_LABEL_COST = 10000
g_node_list = []
g_agent_list = []
g_link_list = []
g_internal_node_id_dict = {}
g_external_node_id_dict = {}
g_number_of_nodes = 0
g_number_of_links = 0
g_number_of_agents = 0


class Node:
        def __init__(self):
                self.node_id = 0
           
                self.x = 0.0
                self.y = 0.0
                self.outgoing_node_list = []


class Link:
        def __init__(self):
                self.BRP_alpha = 0.0
                self.BRP_beta = 0
                self.length = 0.0
                self.flow_volume = 0
                self.link_seq_no = 0
                self.from_node_seq_no = 0
                self.to_node_seq_no = 0
                self.number_of_lanes = 0
                self.lane_cap = 0
                self.speed_limit = 0.0   
                self.travel_time = 0
                self.cost = 0
                self.link_cap = 0
                self.free_flow_travel_time_in_min = 1.0

        def CalculateBRPFunction(self):
                self.link_cap = self.number_of_lanes*self.lane_cap
                self.travel_time = self.free_flow_travel_time_in_min*(1 + self.BRP_alpha*((self.flow_volume / max(0.00001, self.link_cap))**self.BRP_beta))
                self.cost = self.travel_time
    
 
class Agent:
        def __init__(self):
                self.agent_id = 0
                self.origin_node_id = 0
                self.destination_node_id = 0
                self.departure_time = 0.0
                self.PCE_factor = 0
                self.path_cost = 0
                self.path_node_seq_no_list = []
                self.path_link_seq_no_list = []
                                
def g_ReadInputData():
        internal_node_seq_no = 0
        global g_number_of_agents 
        global g_number_of_nodes 
        global g_number_of_links 
        with open('input_node.csv','r') as fp:
                lines = fp.readlines()   
                for l in lines[1:]:
                        l = l.strip().split(',')  
                        try:
                                node = Node()
                                g_internal_node_id_dict[int(l[1])] = internal_node_seq_no
                                g_external_node_id_dict[internal_node_seq_no] = int(l[1])
                                node.node_id = internal_node_seq_no
                                internal_node_seq_no += 1
                                
                                node.x = float(l[2])
                                node.y = float(l[3])
                                
                                g_node_list.append(node)
                                g_number_of_nodes += 1
                                if g_number_of_nodes % 1000 == 0:
                                        print('reading {} nodes..'\
                                                .format(g_number_of_nodes))
                        except:
                                print('Bad read. Check file your self')
                print('nodes_number:{}'.format(g_number_of_nodes))
        
        with open('input_link.csv','r') as fl:
                linel = fl.readlines()             
                for l in linel[1:]:
                        l = l.strip().split(',')
                        try:
                                
                                link = Link()
                                link.length = float(l[4])
                                
                                link.link_seq_no = g_number_of_links
                                link.from_node_seq_no = g_internal_node_id_dict[int(l[1])]
                                link.to_node_seq_no = g_internal_node_id_dict[int(l[2])]
                                link.number_of_lanes = int(l[5])
                                link.lane_cap = float(l[7])
                                link.speed_limit = float(l[6])
                                link.BRP_beta = int(l[10])

                                link.BRP_alpha = float(l[9])
                                link.free_flow_travel_time_in_min = link.length / link.speed_limit *60
                                g_node_list[link.from_node_seq_no].outgoing_node_list.append(link) 
                                g_link_list.append(link)  
                                g_number_of_links += 1
                                
                                if g_number_of_links % 8000 == 0:
                                        print('reading {} links..'\
                                                .format(g_number_of_links))   
                        except:
                                print('Bad read. Check file your self')
                print('links_number:{}'.format(g_number_of_links))
                                
        with open('input_agent.csv','r') as fa:
                linea = fa.readlines()
                for l in linea[1:]:
                        l = l.strip().split(',')
                        try:    
                                agent = Agent()
                                agent.agent_id = int(l[0])
                                agent.origin_node_id = g_internal_node_id_dict[int(l[1])]
                                agent.destination_node_id = g_internal_node_id_dict[int(l[2])]
                                agent.departure_time = int(l[3])
                                agent.PCE_factor = int(l[4])
                                agent.path_link_seq_no_list = []
                                agent.path_node_seq_no_list = []
                                g_agent_list.append(agent)
                                g_number_of_agents += 1
                        except:
                                print('Bad read. Check file your self')
                print('agents_number:{}'.format(g_number_of_agents))


class Node2NodeAccessibility:
        def __init__(self):
                self.from_node_no = 0
                self.to_node_no = 0
                self.travel_cost = 0


class Network:
        #global g_number_of_nodes
        #global g_number_of_links

        def allocate(self,number_of_nodes,number_of_links):
                global _MAX_LABEL_COST
                #global g_number_of_agents
                #global g_number_of_links
                self.node_status_array = [0]*number_of_nodes
                self.node_predecessor = [-1]*number_of_nodes
                self.node_label_cost = [_MAX_LABEL_COST]*number_of_nodes
                self.link_predecessor = [-1]*number_of_nodes
                self.link_cost_array = [1.0]*number_of_links
                self.node2node_accessibility_list = list()
                self.link_volume_array = [0.0]*number_of_links

                for j in range(number_of_links):
                        self.link_volume_array[j] = 0.0
                        self.link_cost_array[j] = 1.0
        
        def optimal_label_correcting(self,origin_node,destination_node,departure_time):
                global _MAX_LABEL_COST
                
                if len(g_node_list[origin_node].outgoing_node_list) == 0:
                    return 0
                
                for i in range(g_number_of_nodes): #Initialization for all nodes
                        self.node_status_array[i] = 0  #not scanned
                        self.node_label_cost[i] = _MAX_LABEL_COST
                        self.node_predecessor[i] = -1 #ointer to previous NODE INDEX from the current label at current node and time
                        self.link_predecessor[i] = -1 #pointer to previous NODE INDEX from the current label at current node and time        
                
                self.node_label_cost[origin_node] = departure_time
                SEList = []
                SEList.append(origin_node)

                
                while len(SEList)>0:
                        from_node = SEList[0]
                        del SEList[0]
                            #print('length of outgoing_node_dict of from_node:',len(agent.outgoing_node_dict[from_node]))
                        for k in range(len(g_node_list[from_node].outgoing_node_list)):
                                to_node = g_node_list[from_node].outgoing_node_list[k].to_node_seq_no
                                #print('to_node:',to_node)
                                b_node_updated = False
                                new_to_node_cost = self.node_label_cost[from_node] + self.link_cost_array[g_node_list[from_node].outgoing_node_list[k].link_seq_no]
                                #print('new_to_node cost:',new_to_node_cost,'from_node_label_cost:',self.node_label_cost[from_node],'to_node_label_cost:',self.node_label_cost[to_node])
                                if (new_to_node_cost < self.node_label_cost[to_node]):  #we only compare cost at the downstream node ToID at the new arrival time t

                                        # update cost label and node/time predecessor

                                        self.node_label_cost[to_node] = new_to_node_cost
                                        #print('new_to_node_label_cost:',self.node_label_cost[to_node])
                                        self.node_predecessor[to_node] = from_node  #pointer to previous physical NODE INDEX from the current label at current node and time
                                        self.link_predecessor[to_node] = g_node_list[from_node].outgoing_node_list[k].link_seq_no  #pointer to previous physical NODE INDEX from the current label at current node and time

                                        b_node_updated = True
                                        
                                        SEList.append(to_node)
                                        #print('SEList after:',SEList)
                                        self.node_status_array[to_node] = 1
                        #print('from_node:',from_node,'to_node:',to_node,'from_node_label_cost:',self.node_label_cost[from_node],'to_node_label_cost_original:',self.node_label_cost[to_node],'new_to_node cost:',new_to_node_cost,)

                        

                if (destination_node >= 0 and self.node_label_cost[destination_node] < _MAX_LABEL_COST):
                    return 1
                elif (destination_node == -1):
                    return 1 # one to all shortest path
                else: 
                    return -1



        def find_path_for_agents(self,iteration_no):
                global g_number_of_links
                for s in range(g_number_of_links):
                        network.link_volume_array[s] = 0

                global g_agent_list      
                #step 1: find shortest path if needed 
                for i in range(len(g_agent_list)):
                        residual = i % (iteration_no + 1)
                        if (residual != 0):  #no need to compute a new path at this iteration
                            continue #that is, it will reuse the path from the previous iteration, stored at p_agent->path_link_seq_no_vector.
                        #else move to the next line for finding the shortest path 
                        g_agent_list[i].path_link_seq_no_list = []
                        g_agent_list[i].path_node_seq_no_list = []
                        #step 2 buil SP tree
                        #print('agent.origin_node_id:',g_agent_list[i].origin_node_id,'agent.destination_node_id:',g_agent_list[i].destination_node_id,'agent.depature_time:',g_agent_list[i].departure_time)
                        return_value = self.optimal_label_correcting(g_agent_list[i].origin_node_id, g_agent_list[i].destination_node_id, g_agent_list[i].departure_time)
                        #step 3 find the destination node

                        #print('return_value:',return_value)
                        if (return_value == -1):
                            print('agent ',i,'can not find destination node')
                            continue
                        #write out path

                        current_node_seq_no = g_agent_list[i].destination_node_id
                        g_agent_list[i].path_cost = self.node_label_cost[g_agent_list[i].destination_node_id]
                        
                        while (current_node_seq_no>=0):
                            if (current_node_seq_no >= 0):  #this is valid node 
                                current_link_seq_no = self.link_predecessor[current_node_seq_no]
                            
                                if(current_link_seq_no>=0):
                                    g_agent_list[i].path_link_seq_no_list.append(current_link_seq_no)      

                            
                                g_agent_list[i].path_node_seq_no_list.append(g_external_node_id_dict[current_node_seq_no])

                            current_node_seq_no = self.node_predecessor[current_node_seq_no]
                        #print('agent id:',i,'from_node:',g_agent_list[i].origin_node_id,'to_node:',g_agent_list[i].destination_node_id,'path_link:',g_agent_list[i].path_link_seq_no_list,'path_node',g_agent_list[i].path_node_seq_no_list)

                #step 2:  scan the shortest path to compute the link volume, 
                
                writer = csv.writer(open('output_agent.csv','w+'))    
                writer.writerow(['iteration_no','agent_id','from_node','to_node','path_link_seq','path_node_seq'])
                for i in range(len(g_agent_list)):

                        for j in range(len(g_agent_list[i].path_link_seq_no_list)):#for each link in the path of this agent

                                link_seq_no = g_agent_list[i].path_link_seq_no_list[j]
                                self.link_volume_array[link_seq_no] += g_agent_list[i].PCE_factor 
                        try:
                            
                                line = [iteration_no,i,g_external_node_id_dict[g_agent_list[i].origin_node_id],g_external_node_id_dict[g_agent_list[i].destination_node_id],g_agent_list[i].path_link_seq_no_list,g_agent_list[i].path_node_seq_no_list]
                                
                                writer.writerow(line)
                        except:
                                print('wirte out agent csv failure!')

        def optimal_label_correcting_for_all_nodes(self):
                global g_node_list
                print('writing node accessibility of all node pairs to output_node_accessibility.csv...')
                writer1 = csv.writer(open('output_node_accessibility.csv','w+'))
                writer1.writerow(['from_node','to_node','travel_cost'])

                for i in range(len(g_node_list)): #Initialization for all nodes
                  
                        origin = i
                        return_value = self.optimal_label_correcting(origin, -1, 0);
                        #print('return_value:',return_value)

                        if return_value >= 1:
                            
                            for j in range(g_number_of_nodes):
                                    if (j != origin and self.node_label_cost[j] < _MAX_LABEL_COST):
                                        element = Node2NodeAccessibility()
                                        element.from_node_no = origin
                                        element.to_node_no = j
                                        element.travel_cost = self.node_label_cost[j]
                                        #print('from node:',element.from_node_no,'to_node:',element.to_node_no,'travel_cost:',self.node_label_cost[j])
                                        self.node2node_accessibility_list.append(element)
                                        try:
                                                line = [g_external_node_id_dict[element.from_node_no],g_external_node_id_dict[element.to_node_no],self.node_label_cost[j]]
                                                
                                                writer1.writerow(line)
                                                #print('4')
                                        except:
                                                print('wirte out node accessiblity failure!')




if __name__=='__main__':
        
        print('Reading data......')
        g_ReadInputData()
        
        print('Allocating memory......')
        network = Network()
        network.allocate(g_number_of_nodes,g_number_of_links)

        #print('Node Label Cost Original:--------------------------------------------------------------------------!')
        #print(network.node_label_cost[0],network.node_label_cost[1],network.node_label_cost[2])

        print('Finding shortest path for all agents......')

        for i in range(10):
                print('iteration_no',i,'......')
                for l in range(g_number_of_links):
                        g_link_list[l].CalculateBRPFunction()
                        network.link_cost_array[l] = g_link_list[l].cost

                network.find_path_for_agents(i)     
                        
                
                for k in range(g_number_of_links):
                        g_link_list[k].flow_volume = 0.0
                        g_link_list[k].flow_volume += network.link_volume_array[k]
                        #print('link id:',k,'link cost:',network.link_cost_array[l],'link_volume',network.link_volume_array[k],'BRP:FFTT',g_link_list[l].free_flow_travel_time_in_min)

        #print('Node Label Cost after find path for all agents:')
        #print(network.node_label_cost[0],network.node_label_cost[1],network.node_label_cost[2])
        print('writing all agents path to output_agent.csv......')
        
        
        print('Finding shortest path for all nodes......')
        network.optimal_label_correcting_for_all_nodes()
        '''
        print('Node label cost for all nodes pairs......')
        print(network.node_label_cost[0],network.node_label_cost[1],network.node_label_cost[2])
        '''
        




