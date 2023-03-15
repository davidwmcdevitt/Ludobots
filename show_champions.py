from tournament import *

''''''''''''''''''''''''''''''''''''''''''''''''''
'''To show each method, uncomment one variable'''

#method = "organic"
method = "lstm"

''''''''''''''''''''''''''''''''''''''''''''''''''

if method == "organic":
    
    with open('training_data/organicWiggle_1678846744.pkl', 'rb') as f:
        left_dna = pickle.load(f)
              
    with open('training_data/organicWiggle_1678846743.pkl', 'rb') as f:
        right_dna = pickle.load(f)

if method == "lstm":
    
    vocab = list(range(1000))
    
    model = brain_model(vocab, context_size = 296, embedding_size = 100 ,hidden_size = 128, num_layers = 2)
    saved_params = torch.load('training_data/brain_model_1678849905.pt')
    model.load_state_dict(saved_params)
    
    model.eval()
    initial_token = torch.tensor([1]) 
    
    print("Generating Left...")
    left_dna = []
    with torch.no_grad():
       for i in range(296):
            output = model(initial_token)
            k = 2
            _, predicted = torch.max(output, dim=0)
            if len(left_dna) > 0 :
              if predicted == left_dna[len(left_dna)-1]:
                predicted = torch.topk(output, k =k).indices[k-1]
                k+=1
            left_dna.append(predicted.item())
            initial_token = torch.cat((initial_token,predicted.unsqueeze(0)))
      
    right_dna = []
    
    print("Generating Right...")
    with torch.no_grad():
       for i in range(296):
            output = model(initial_token)
            k = 2
            _, predicted = torch.max(output, dim=0)
            if len(right_dna) > 0 :
              if predicted == right_dna[len(right_dna)-1]:
                predicted = torch.topk(output, k =k).indices[k-1]
                k+=1
            right_dna.append(predicted.item())
            initial_token = torch.cat((initial_token,predicted.unsqueeze(0)))
                  
    


leftroot_neuron, rightroot_neuron, leftHead_index, rightHead_index = build_challenger(left_dna, right_dna)

leftAlive, rightAlive = start_world(direct= False)
sensors, motors, brain, robot = prepare_matchup()
 
t = 0
while leftAlive == True and rightAlive == True:
    p.stepSimulation()
    
    sense(t, sensors, robot, leftHead_index, rightHead_index)
    think(brain)
    act(t, motors, brain, robot,leftroot_neuron, rightroot_neuron)
    #leftAlive, rightAlive = check(robot, leftHead_index, rightHead_index)
    #time.sleep(1/5000)
    
    t+=1