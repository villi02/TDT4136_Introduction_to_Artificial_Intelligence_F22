# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"
        return successorGameState.getScore()

def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def minValue(self, gameState, depth, agentIndex):
        """
        Returns min value for a state
        """
        # Return if game is won or lost
        if ( gameState.isWin() or gameState.isLose()):
            return self.evaluationFunction(gameState)

        minEval = 1000000
        for action in gameState.getLegalActions(agentIndex):
            nextState = gameState.generateSuccessor(agentIndex, action)

            # Since it's the last ghost making a move, then we'll not call minValue again, we'll instead call maxvalue as this will activate pacman again, as this will be a new depth to explore
            if (agentIndex == (gameState.getNumAgents()-1)):
                minEval = min(minEval, self.maxValue(nextState, depth))

            # Otherwise call minEval on the next ghosts based on the current ones move
            else:
                minEval = min(minEval, self.minValue(nextState, depth, agentIndex+1))

        return minEval

    def maxValue(self, gameState, depth):
        """
        Returns min value for a state
        """
        # Updates depth in maxValue and not minValue because it is only when pacman moves that there is a new depth, and he'll move after all the ghosts have also moved, this avoids so that pacman thinks that he can move inbetween the individual ghosts moving
        thisDepth = depth + 1
        # Return if at this depth or game is won or lost
        if ( thisDepth == self.depth or gameState.isWin() or gameState.isLose()):
            return self.evaluationFunction(gameState)

        maxEval = -1000000
        # Checking future moves based on the current move
        for action in gameState.getLegalActions(0):
            nextState = gameState.generateSuccessor(0, action)
            # starts with agentIndex 1 because it will then recursively iterate through all the agents (who are ghosts)
            maxEval = max(self.minValue(nextState, thisDepth, 1), maxEval)

        return maxEval

    def getBestAction(self, gameState):

        # Set best score to inf low, to then change it to the highest of the possible outcomes
        maxScore = -999999999
        for action in gameState.getLegalActions(0):
            nextState = gameState.generateSuccessor(0, action)
            nextValue = self.maxValue(nextState, 0)
            if maxScore != max(nextValue, maxScore):
                maxScore = nextValue
                nextMove = action

        return nextMove


    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"

        maxScore = -999999999
        for action in gameState.getLegalActions(0):
            nextState = gameState.generateSuccessor(0, action)
            nextScore = self.minValue(nextState, 0, 1)
            if maxScore != max(nextScore, maxScore):
                maxScore = nextScore
                nextMove = action

        return nextMove

        util.raiseNotDefined()

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def minValue(self, gameState, depth, agentIndex, alpha, beta):
        """
        Returns min value for a state
        """
        # Return if game is won or lost, no need to check for depth, as it will not change until pacman moves
        if ( gameState.isWin() or gameState.isLose()):
            return self.evaluationFunction(gameState)

        minEval = 1000000
        for action in gameState.getLegalActions(agentIndex):
            nextState = gameState.generateSuccessor(agentIndex, action)
            # Since it's the last ghost making a move, then we'll not call minValue again, we'll instead call maxvalue as this will activate pacman again, as this will be a new depth to explore
            if (agentIndex == (gameState.getNumAgents()-1)):
                minEval = min(minEval, self.maxValue(nextState, depth, alpha, beta))
                beta = min(minEval, beta)
                if beta < alpha:
                    return minEval
            else:
                minEval = min(minEval, self.minValue(nextState, depth, agentIndex+1, alpha, beta))
                beta = min(minEval, beta)
                if beta < alpha:
                    return minEval
        return minEval

    def maxValue(self, gameState, depth, alpha, beta):
        """
        Returns min value for a state
        """
        # Updates depth in maxValue and not minValue because it is only when pacman moves that there is a new depth, and he'll move after all the ghosts have also moved, this avoids so that pacman thinks that he can move inbetween the individual ghosts moving
        thisDepth = depth + 1
        # Return if at this depth or game is won or lost
        if ( thisDepth == self.depth or gameState.isWin() or gameState.isLose()):
            return self.evaluationFunction(gameState)

        maxEval = -1000000
        # Checking future moves based on the current move
        for action in gameState.getLegalActions(0):
            nextState = gameState.generateSuccessor(0, action)
            # starts with agentIndex 1 because it will then recursively iterate through all the agents (who are ghosts)
            maxEval = max(self.minValue(nextState, thisDepth, 1, alpha, beta), maxEval)
            alpha = max(alpha, maxEval)
            if beta < alpha:
                break

        return maxEval

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"

        maxScore = -999999999
        startAlpha = -999999999
        startBeta = 999999999
        for action in gameState.getLegalActions(0):
            nextState = gameState.generateSuccessor(0, action)
            nextScore = self.minValue(nextState, 0, 1, startAlpha, startBeta)
            if maxScore != max(nextScore, maxScore):
                maxScore = nextScore
                nextMove = action
            if startBeta != max(startBeta, nextScore):
                return nextMove
            startAlpha = max(startAlpha, nextScore)
        return nextMove

        util.raiseNotDefined()

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

# Abbreviation
better = betterEvaluationFunction
