# baselineTeam.py
# ---------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


# baselineTeam.py
# ---------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from captureAgents import CaptureAgent
import distanceCalculator
import random, time, util, sys
from game import Directions
import game
from util import nearestPoint

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'AstarOffensiveAgent', second = 'QLearningDefensiveAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########
class DummyAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''


  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    actions = gameState.getLegalActions(self.index)

    '''
    You should change this in your own agent.
    '''

    return random.choice(actions)

  def getSuccessor(self, gameState, action):
    #find the next state
    successor = gameState.generateSuccessor(self.index,action)
    position = successor.getAgentState(self.index).getPosition()
    if position != nearestPoint(position):
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

class AstarOffensiveAgent(DummyAgent):
    '''
    This is an offensive agent to eat opponent's foods,
    which applies A* with heuristic function.
    '''
    def registerInitialState(self, gameState):
        '''
        This is a function to initialize arguments.
        '''
        self.start = gameState.getAgentPosition(self.index)
        self.counter = 0
        self.goal = []
        self.lastGhostsPos = {}
        self.counter = {}
        CaptureAgent.registerInitialState(self, gameState)
        for opponentIndex in self.getOpponents(gameState):
            self.counter[opponentIndex] = 0

    def isGoal(self, gameState):
        '''
        This function is used in A* search to judge the goal state
        '''
        if gameState.getAgentState(self.index).getPosition() in self.goal:
            return True
        else:
            return False

    def chooseAction(self, gameState):
        '''
        Return the best action currently to move according to the state.
        '''
        myPos = gameState.getAgentState(self.index).getPosition()
        # record the positions of the opponent ghosts when they appear in the last time
        for opponentIndex in self.getOpponents(gameState):
            if gameState.getAgentState(opponentIndex).getPosition() is not None:
                ghostPosition = gameState.getAgentState(opponentIndex).getPosition() # can see
                scaredTimer = gameState.getAgentState(opponentIndex).scaredTimer
                if gameState.getAgentState(opponentIndex).isPacman == False: # is ghost
                    if scaredTimer <= 9:
                        self.lastGhostsPos[opponentIndex] = ghostPosition
                        self.counter[opponentIndex] = 0
                    elif scaredTimer >= 9:
                        if opponentIndex in self.lastGhostsPos:
                            self.lastGhostsPos.pop(opponentIndex)
                            self.counter[opponentIndex] = 0
                else: # if it is not ghost, remove it
                    if opponentIndex in self.lastGhostsPos:
                        self.lastGhostsPos.pop(opponentIndex)
            else:
                self.counter[opponentIndex] += 1
                if self.counter[opponentIndex] % 16 == 0 and opponentIndex in self.lastGhostsPos:
                    self.lastGhostsPos.pop(opponentIndex)

        # calculate the minimum distance to one ghost
        distancesToGhost = []
        if len(self.lastGhostsPos) != 0:
            for opponentIndex in self.getOpponents(gameState):
                if opponentIndex in self.lastGhostsPos:
                    distancesToGhost.append(self.getMazeDistance(myPos, self.lastGhostsPos[opponentIndex]))
            distanceToGhost = min(distancesToGhost)
        else:
            distanceToGhost = 999

        # record the maximum time left for opponent to be scared
        scaredTimer = 0
        opponentIndex = self.getOpponents(gameState)
        for oppo in opponentIndex:
            currentTimer = gameState.getAgentState(oppo).scaredTimer
            if currentTimer > 0:
                scaredTimer = currentTimer

        # how many foods left to eat
        foodLeft = len(self.getFood(gameState).asList())

        # distance to the nearest food
        minFoodDis = 999
        for food in self.getFood(gameState).asList():
            dis = self.getMazeDistance(myPos, food)
            if dis < minFoodDis:
                minFoodDis = dis

        # if (1) a ghost that is not scared is nearby
        # (2) or there is fewer than 2 foods left
        # (3) or pacman has eat more than 4 foods, and is or is going to be dangerous ghost, and has no food nearby within one step
        # then find a path to home (go back and escape)
        # else find a path to food
        carriedFood = gameState.getAgentState(self.index).numCarrying
        if distanceToGhost <= 5 or foodLeft <= 2 or (carriedFood >= 4 and scaredTimer <= 9 and minFoodDis >= 2) and gameState.getAgentState(self.index).isPacman: # go home
            self.goal = [self.start] # goal1: home postion
            for element in self.getCapsules(gameState): # goal2: capsules
                self.goal.append(element)
            myFood = self.getFoodYouAreDefending(gameState).asList()
            for food in myFood: # goal3: foods in team area
                self.goal.append(food)
        else: # eat foods
            self.goal = self.getFood(gameState).asList()

        # find the path using A* search
        bestAction = self.aStarSearch(gameState)
        return bestAction

    def aStarSearch(self, gameState):
        '''
        A* search with heuristic function
        Return the first action in the best path to goal
        '''
        closed = [] # visited positions list
        priorityQueue = util.PriorityQueue()
        initialState = gameState
        priorityQueue.push([initialState, [], 0], 0)
        while not priorityQueue.isEmpty():
            currentState, actionsToCurrent, costToCurrent = priorityQueue.pop()
            if not currentState.getAgentState(self.index).getPosition() in closed: # expand unvisited state
                if self.isGoal(currentState): # whether this is goal state
                    if len(actionsToCurrent) == 0:  # if no action returned, then return the nearest action to home.
                        legalActions = currentState.getLegalActions(self.index)
                        result = legalActions[0]
                        minDistance = 999
                        for action in legalActions:
                            successor = self.getSuccessor(gameState,action)
                            position = successor.getAgentState(self.index).getPosition()
                            distanceToStart = self.getMazeDistance(position,self.start)
                            if distanceToStart < minDistance:
                                minDistance = distanceToStart
                                result = action
                        return result
                    else:
                        return actionsToCurrent[0] # first action to home
                closed.append(currentState.getAgentState(self.index).getPosition())
                actions = currentState.getLegalActions(self.index)
                for action in actions:
                    nextState = self.getSuccessor(currentState, action)
                    if not nextState.getAgentState(self.index).getPosition() in closed: # push unvisited state
                        actionsToNext = actionsToCurrent + [action]
                        costToNext = costToCurrent + 1 # steps from initial position to current position
                        costWithHeuristic = costToNext + self.heuristic(nextState, costToNext) # this value is the priority
                        priorityQueue.push([nextState, actionsToNext, costToNext], costWithHeuristic) # The nearer from ghost, it has lower priority to be expanded

    def heuristic(self, gameState, costToNext):
        '''
        heuristic function used in A* search
        Return a value that has a negative correlation with the distance between pacman and opponent ghost.
        The shorter the distance between pacman and ghost, the biger the value it returnself.
        As a result, it will cause a lower priority to be expanded
        '''
        thisPos = gameState.getAgentState(self.index).getPosition()

        # calculate the distance to the nearest ghost
        distancesToGhost = []
        value = 0
        if len(self.lastGhostsPos) != 0:
            for opponentIndex in self.getOpponents(gameState):
                if opponentIndex in self.lastGhostsPos:
                    if self.getMazeDistance(thisPos, self.lastGhostsPos[opponentIndex]) <= 5:
                        value += (6 - self.getMazeDistance(thisPos, self.lastGhostsPos[opponentIndex])) * 1000

        # return a heuristic value: the longer the distance, the larger the value
        return value


class QLearningDefensiveAgent(DummyAgent):
  '''
  This class is a defensive agent class based on Q-learning
  The main point of this class is the design of reward of each action
  The reward of each action is designed to related to several fectures:
    1. "isPacman" means if the agent would become a pacman after doing this action.
    2. "distanceToFoodDiff" means if this action could make the average distance to
       foods under defending smaller.
    3. "viewMoreEnemies" means if this action could make more enemies into my view
    4. "closerEnemy" means if this action could make me closer to the enemy insight
    5. "distanceToLatestEatenFood" means if this action could make me closer to the position of
        the latest food eaten by the enemy so far.
    6. "stop" means if this action is "STOP"
    7. "reverse" means if this action make the direction of the agent reverse
  Generally, we want to : 1. avoid the defender from becoming a pacman
                          2. be close to the positon which has smaller average distance to all
                             the food I defending.
                          3. go to the position where can view more enemies.
                          4. be close to the cloest enemy in sight.
                          5. be close to the position of the latest food being eaten.
                          6. never stop unless I am scared and have to avoid meeting the enemy.
                          7. avoid to reverse the direction to some extent
  The max value of all the Q values of each position represents the value of that position so far
  which means if that position is a worthy position. This is a very important value when I patrol.
  This means I will patrol those positions where the enemy have ever appeared a little more.
  '''

  def initParameters(self, epsilon = 0.00, alpha = 0.8, gamma = 0.4):
    '''
    epsilon -> exploration rate (epsilon-greedy)
    alpha -> learning rate
    gamma -> discount factor
    previousFoodList -> the defended food list of the previous step
    defendFoodList -> the defended fodd list of the current step
    latestEatenFood -> the latest food to be eaten by the enemy
    '''
    self.epsilon = float(epsilon)
    self.alpha = float(alpha)
    self.discount = float(gamma)
    self.qTable = util.Counter()

  def registerInitialState(self, gameState):

    self.previousFoodList = []
    self.defendFoodList = self.getFoodYouAreDefending(gameState).asList()
    self.latestEatenFood = list(set(self.previousFoodList).difference(set(self.defendFoodList)))
    self.ghostEnemy = []
    self.foodList = self.getFood(gameState).asList()
    self.initParameters()
    DummyAgent.registerInitialState(self,gameState)

  def chooseAction(self,gameState):
    #refresh the foodlists
    self.previousFoodList = self.defendFoodList
    self.defendFoodList = self.getFoodYouAreDefending(gameState).asList()

    # If the enemy ate a new food between my two steps refresh the self.latestEatenFood
    newlyEatenFood = list(set(self.previousFoodList).difference(set(self.defendFoodList)))
    if len(newlyEatenFood) != 0:
      self.latestEatenFood = newlyEatenFood

    #update the Q(s,a) for each legal actions
    actions = gameState.getLegalActions(self.index)
    for action in actions:
      self.updateQ(gameState,action)

    #choose the best action based on the Q(s,a) using epsilon-greedy
    result = self.getBestAction(gameState)
    if util.flipCoin(self.epsilon): result = random.choice(actions)
    return result

  def updateQ(self, gameState, action):

    currentState = gameState.getAgentState(self.index)
    currentPosition = currentState.getPosition()
    successor = self.getSuccessor(gameState, action)
    nextState = successor.getAgentState(self.index)
    nextPosition = nextState.getPosition()

    self.foodList = self.getFood(gameState).asList()

    #enemies insight of the current position
    currentOpponents = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
    currentEnemies = [enemy for enemy in currentOpponents if enemy.isPacman and enemy.getPosition() != None]
    self.ghostEnemy  = [enemy for enemy in currentOpponents if enemy.isPacman == False and enemy.getPosition() != None]
    #enemies insight of the next position
    nextOpponents = [successor.getAgentState(i) for i in self.getOpponents(successor)]
    nextEnemies = [enemy for enemy in nextOpponents if enemy.isPacman and enemy.getPosition() != None]

    #The position of the lastestEatenFood would be reset after I get to the around position of it.
    if len(self.latestEatenFood) != 0 and self.getMazeDistance(currentPosition,self.latestEatenFood[0]) < 3:
      self.latestEatenFood = []

    #set the features and their weights to calculate the reward of this action
    rewardFeatures = util.Counter()

    rewardWeights = {'isPacman': -600, 'distanceToFoodDiff':110, 'viewMoreEnemies': 100, 'closerEnemy': 500, 'distanceToLatestEatenFood':400, 'stop': -150, 'reverse': -100}

    #The first feature "isPacman"
    rewardFeatures['isPacman'] = 0
    if nextState.isPacman: rewardFeatures['isPacman'] = 1

    #The second feature "distanceToFoodDiff" which means if this action
    #could make the average distance to foods under defending smaller.
    #The default value is 0
    currentFoodDistances = 0
    nextFoodDistances = 0
    for defendFood in self.defendFoodList:
      currentFoodDistances += self.getMazeDistance(currentPosition, defendFood)
      nextFoodDistances += self.getMazeDistance(nextPosition, defendFood)
    if len(self.defendFoodList) != 0:
        currentAverage = currentFoodDistances/len(self.defendFoodList) #average distance
        nextAverage = nextFoodDistances/len(self.defendFoodList) #average distance
        if currentAverage > nextAverage:
            rewardFeatures['distanceToFoodDiff'] = 1.0
        else:
            rewardFeatures['distanceToFoodDiff'] = -1.0

    '''
    if (len(currentEnemies) != 0 or len(nextEnemies) != 0) and notScared == 1:
      rewardFeatures['distanceToFoodDiff'] = 0.5
  `'''
    #The third feature which means if this action could make more enemies into my view
    if len(nextEnemies) > len(currentEnemies):
      rewardFeatures['viewMoreEnemies'] = 1

    #The fourth feature which means if this action could make me closer to the cloest enemy insight
    #when both of the current position and the next position could see the enemy
    if len(currentEnemies) > 0 and len(nextEnemies) > 0:
      currentDistances = [self.getMazeDistance(currentPosition, enemy.getPosition()) for enemy in currentEnemies]
      currentMinDistance = min(currentDistances)
      nextDistances = [self.getMazeDistance(nextPosition, enemy.getPosition()) for enemy in nextEnemies]
      nextMinDistance = min(nextDistances)
      #when the next position is closer to the enemy and I'm not scared,
      #I would get a positive reward for this feature, which means I would prefer
      #to get to the next position by applying the action. However, if I'm scared,
      #the reward for this feature would be negative and I will prefer to go far away
      #from the enemy.
      if nextMinDistance < currentMinDistance:
        if currentState.scaredTimer < currentMinDistance - 1:
          rewardFeatures['closerEnemy'] = 1.0
        else:
          rewardFeatures['closerEnemy'] = -1.0

    #when I would meet the enemy on the next positon
    if nextPosition in [enemy.getPosition() for enemy in currentEnemies]:
      rewardFeatures['closerEnemy'] = 1.0

    #The fifth feature which means if this action could make me closer to the position of
    #the latest food eaten by the enemy so far.
    #I want to be closer to latestEatenFood, because enemy is there even he may be not in my view
    if len(self.latestEatenFood) > 0:
      currentDistance = min([self.getMazeDistance(currentPosition, food) for food in self.latestEatenFood])
      nextDistance = min([self.getMazeDistance(nextPosition, food) for food in self.latestEatenFood])
      if nextDistance < currentDistance:
        if currentState.scaredTimer < currentDistance - 1:
          rewardFeatures['distanceToLatestEatenFood'] = 1.0

    #The sixth feature which means I want to avoid from stopping to some extent
    if action == Directions.STOP: rewardFeatures['stop'] = 1

    #The last feature which means that I don't want the agent to reverse its direction
    toward = gameState.getAgentState(self.index).configuration.direction
    reverse = Directions.REVERSE[toward]
    if action == reverse:
      rewardFeatures['reverse'] = 1
    #elif action == toward:
    #rewardFeatures['reverse'] = -0.1

    #sepcial conditions:
    #if the nextPosition has a food, at the same time there are no enemies around
    #I will come to eat it
    currentDistanceToFood = min([self.getMazeDistance(currentPosition,food) for food in self.foodList])
    nextDistanceToFood = min([self.getMazeDistance(nextPosition,food) for food in self.foodList])
    if len(self.ghostEnemy) == 0 and len(self.latestEatenFood) == 0 and nextDistanceToFood < currentDistanceToFood and nextDistanceToFood < 4:
        if currentState.numCarrying < 3:
            rewardFeatures['isPacman'] = -0.5
    if len(self.latestEatenFood) == 0 and nextDistanceToFood < currentDistanceToFood and len(self.ghostEnemy) == 0:
        if currentState.numCarrying < 4:
            rewardFeatures['isPacman'] = -0.2

    reward = rewardFeatures * rewardWeights

    #calculate Q(s,a)
    if len(successor.getLegalActions(self.index)) == 0:
      differ = reward
    else:
      differ = reward + (self.discount * self.getValue(successor))

    factorOne = (1 - self.alpha) * self.getQValue(currentPosition,action)
    factorTwo = self.alpha * differ
    #update Q table
    self.qTable[(currentPosition,action)] = factorOne + factorTwo

  def getBestAction(self, gameState):
    #select one of the actions which have the highest Q(s,a)
    currentPosition = gameState.getAgentState(self.index).getPosition()
    maxQValue = self.getValue(gameState)
    actions = gameState.getLegalActions(self.index)
    bestActions = [action for action in actions if self.getQValue(currentPosition,action) == maxQValue]
    bestAction = random.choice(bestActions)
    #avoid always walking by cycle
    self.qTable[(currentPosition,bestAction)] -= 40

    #In practice, the agent may keep going back and forth in one area
    #so I decrease the Q(s,a) for the reverse action of the selected action to avoid this
    successor = self.getSuccessor(gameState,bestAction)
    nextPosition = successor.getAgentState(self.index).getPosition()
    reverse = Directions.REVERSE[bestAction]
    if reverse in actions: self.qTable[(nextPosition,reverse)] -= 50
    return bestAction

  def getQValue(self, position, action):
    return self.qTable[(position,action)]

  def getValue(self, gameState):
    #return the max Q(s,a) of a state s.
    position = gameState.getAgentState(self.index).getPosition()
    actions = gameState.getLegalActions(self.index)
    qValues = []

    for action in actions:
      qValues.append(self.getQValue(position,action))

    if len(actions) == 0:
      return 0.0
    else:
      return max(qValues)
