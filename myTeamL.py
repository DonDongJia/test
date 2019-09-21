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
               first = 'OffensiveReflexAgent', second = 'DefensiveReflexAgent'):
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

class ReflexCaptureAgent(CaptureAgent):
  """
  A base class for reflex agents that chooses score-maximizing actions
  """

  def registerInitialState(self, gameState):
    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)

  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """
    actions = gameState.getLegalActions(self.index)

    # You can profile your evaluation time by uncommenting these lines
    # start = time.time()
    values = [self.evaluate(gameState, a) for a in actions]
    # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

    maxValue = max(values)
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]

    foodLeft = len(self.getFood(gameState).asList())

    if foodLeft <= 2 or gameState.getAgentState(self.index).numCarrying==2:
      bestDist = 9999
      for action in actions:
        successor = self.getSuccessor(gameState, action)
        pos2 = successor.getAgentPosition(self.index)
        dist = self.getMazeDistance(self.start,pos2)
        if dist < bestDist:
          bestAction = action
          bestDist = dist
      return bestAction

    return random.choice(bestActions)

  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

  def evaluate(self, gameState, action):
    """
    Computes a linear combination of features and feature weights
    """
    features = self.getFeatures(gameState, action)
    weights = self.getWeights(gameState, action)
    return features * weights


class OffensiveReflexAgent(CaptureAgent):
    def registerInitialState(self, gameState):
        self.start = gameState.getAgentPosition(self.index)
        CaptureAgent.registerInitialState(self, gameState)

    def getSuccessor(self, gameState, action):
        successor = gameState.generateSuccessor(self.index, action)
        position = successor.getAgentState(self.index).getPosition()
        if position != nearestPoint(position):
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def isGoal(self, gameState):
        if gameState.getAgentState(self.index).getPosition() in self.goal:
            return True
        else:
            return False

    def chooseAction(self, gameState):
        self.escape = False

        self.myPos = gameState.getAgentState(self.index).getPosition()

        ghostsPos = []
        distancesToGhost = []
        for opponentIndex in self.getOpponents(gameState):
            if gameState.getAgentState(opponentIndex).isPacman==False:
                if not gameState.getAgentState(opponentIndex).getPosition() is None:
                    ghostsPos.append(gameState.getAgentState(opponentIndex).getPosition())
        for ghostPos in ghostsPos:
            if ghostPos is None:
                distancesToGhost.append(9999)
            else:
                distancesToGhost.append(self.getMazeDistance(self.myPos, ghostPos))
        if len(distancesToGhost):
            distanceToGhost = min(distancesToGhost)
        else:
            distanceToGhost = 9999

        foodList = self.getFood(gameState).asList()
        if len(foodList) > 0:
            minFoodDis = 9999
            for food in foodList:
                if minFoodDis > self.getMazeDistance(self.myPos, food):
                    minFoodDis = self.getMazeDistance(self.myPos, food)
                    self.foodToEat = food

        if distanceToGhost < 6:
            self.escape = True
            self.goal = [self.start]
            for element in self.getCapsules(gameState):
                self.goal.append(element)
        else:
            self.escape = False
            self.goal = self.getFood(gameState).asList()

        # print str(self.goal)+"*****************************************************"
        # print str(gameState.getAgentState(self.index).getPosition())+"*********************************"
        actions = gameState.getLegalActions(self.index)
        bestActions = self.aStarSearch(gameState)

        foodLeft = len(self.getFood(gameState).asList())

        scaredTimer = 0
        opponentIndex = self.getOpponents(gameState)
        for oppo in opponentIndex:
            currentTimer = gameState.getAgentState(oppo).scaredTimer
            if currentTimer!=0:
                scaredTimer=currentTimer

        if foodLeft <= 2 or (gameState.getAgentState(self.index).numCarrying >= 5) and (scaredTimer==0):
            bestDist = 9999
            for action in actions:
                successor = self.getSuccessor(gameState, action)
                pos2 = successor.getAgentPosition(self.index)
                dist = self.getMazeDistance(self.start,pos2)
                if dist < bestDist:
                    bestAction = action
                    bestDist = dist
            return bestAction
            # self.escape = True
            # self.goal = self.start
            # bestActions = self.aStarSearch(gameState)
            # print bestActions[0]
            # return bestActions[0]
        return bestActions[0]

    def aStarSearch(self, gameState):
        closed = []
        priorityQueue = util.PriorityQueue()
        initialState = gameState
        priorityQueue.push([initialState, [], 0], 0)
        while not priorityQueue.isEmpty():
            currentState, actionsToCurrent, costToCurrent = priorityQueue.pop()
            if not currentState.getAgentState(self.index).getPosition() in closed:
                if self.isGoal(currentState):
                    return actionsToCurrent
                closed.append(currentState.getAgentState(self.index).getPosition())
                actions = currentState.getLegalActions(self.index)
                for action in actions:
                    nextState = self.getSuccessor(currentState, action)
                    if not nextState.getAgentState(self.index).getPosition() in closed:
                        actionsToNext = actionsToCurrent + [action]
                        costToNext = costToCurrent + 1
                        costWithHeuristic = costToNext + self.heuristic(nextState)

                        priorityQueue.push([nextState, actionsToNext, costToNext], costWithHeuristic)
        # print "Path not Found"

    def heuristic(self, gameState):
        features = util.Counter()
        foodList = self.getFood(gameState).asList()
        capsule  = self.getCapsules(gameState)
        foodList = foodList + capsule

        features['successorScore'] = len(foodList)

        myPos = gameState.getAgentState(self.index).getPosition()

        # minFoodDis = 9999
        # for food in foodList:
        #     if minFoodDis > self.getMazeDistance(myPos, food):
        #         minFoodDis = self.getMazeDistance(myPos, food)
        #         foodToEat = food
        # features['distanceToFood'] = minFoodDis

        ghostsPos = []
        distancesToGhost = []
        for opponentIndex in self.getOpponents(gameState):
            if gameState.getAgentState(opponentIndex).isPacman==False:
                if not gameState.getAgentState(opponentIndex).getPosition() is None:
                    ghostsPos.append(gameState.getAgentState(opponentIndex).getPosition())
        for ghostPos in ghostsPos:
            if ghostPos is None:
                distancesToGhost.append(999)
            else:
                distancesToGhost.append(self.getMazeDistance(myPos, ghostPos))
        if len(distancesToGhost):
            distanceToGhost = min(distancesToGhost)*10
        else:
            distanceToGhost = 999

        # if distanceToGhost < 6:
        #     features['distanceToGhost'] = 6 - distanceToGhost
        # features['distanceToHome'] = self.getMazeDistance(myPos, self.start)
        # else:
        #     features['distanceToGhost'] = 0
        #     features['distanceToHome'] = 0
        # if self.escape == True:
            # heuristicValue = features*{'successorScore': 0, 'distanceToFood': 0, 'distanceToGhost':0,'distanceToHome':1}
        #     heuristicValue = self.getMazeDistance(myPos, self.start)
        # else:
        #     heuristicValue = features*{'successorScore': 100, 'distanceToFood': 1, 'distanceToGhost':0,'distanceToHome':0}
        return -distanceToGhost
# if distanceToGhost <= 6:
    # list=[]
    # list.append(self.getMazeDistance(myPos, self.start))
    # if self.red:
    #     if gameState.getBlueCapsules():
    #         print gameState.getBlueCapsules()
    #         for cap in gameState.getBlueCapsules():
    #             list.append(self.getMazeDistance(myPos, cap))
    # else:
    #     if gameState.getRedCapsules():
    #         for cap in gameState.getBlueCapsules():
    #             list.append(self.getMazeDistance(myPos, cap))

    # features['distanceToHomeOrCap'] = min(list)
#     features['distanceToHomeOrCap'] = self.getMazeDistance(myPos, self.start)
# else:
#     features['distanceToHomeOrCap'] = 0


class DefensiveReflexAgent(ReflexCaptureAgent):
  """
  A reflex agent that keeps its side Pacman-free. Again,
  this is to give you an idea of what a defensive agent
  could be like.  It is not the best or only way to make
  such an agent.
  """

  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)

    myState = successor.getAgentState(self.index)
    myPos = myState.getPosition()

    # Computes whether we're on defense (1) or offense (0)
    features['onDefense'] = 1
    if myState.isPacman: features['onDefense'] = 0

    # Computes distance to invaders we can see
    enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    features['numInvaders'] = len(invaders)
    if len(invaders) > 0:
      dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
      features['invaderDistance'] = min(dists)

    if action == Directions.STOP: features['stop'] = 1
    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1

    return features

  def getWeights(self, gameState, action):
    return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -2}
