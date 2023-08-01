#include <Arduino.h>

#define NUM_SIMS 100
#define PERFECT_SCORE 24
#define WARNING_POSITION 21

typedef enum game_end_t {
  WIN,
  LOSE,
  WARNING,
  PERFECT
} game_end_t;


uint8_t getNextMove();
uint8_t getScore();
void generateLoseGame();
void generatePerfectGame();
void generateWarningGame();
void generateEasyWin();

void outputMoves();
uint8_t randomDistance();
uint8_t randomDistanceMin12();
uint8_t randomDistanceWeighted();
int preSelectedDistance();
void selectGameType();

uint8_t turn = 0;

//select randomizing function here
uint8_t getNextMove() {
  //return randomDistance();
  return randomDistanceMin12();
  //return randomDistanceWeighted();
  //return preSelectedDistance();
}




void resetGame() {
  Serial.println("reseting turn to zero");
  turn = 0;
}

void addOneTurn() {
  turn++;
}

void simulateGame() {
  uint8_t scores[NUM_SIMS];
  uint8_t moves[NUM_SIMS][3];
  game_end_t wins[NUM_SIMS];
  int gamesWon = 0;
  int gamesLost = 0;
  int gamesWarning = 0;
  int gamesPerfect = 0;
  for(int i = 0; i < NUM_SIMS; i ++) {
    scores[i] = 0;
    for(int j = 0; j < 3; j++) {
      int distance = getNextMove();
      scores[i] += distance;
      moves[i][j] = distance;
    }
    if(scores[i] >= 25) {
      wins[i] = LOSE;
    } else if(scores[i] == PERFECT_SCORE) {
      wins[i] = PERFECT;
    }else if(scores[i] >= 21) {
      wins[i] = WARNING;
    } else {
      wins[i] = WIN;
    }
  }

  for(int i = 0; i < NUM_SIMS; i ++) {
    //if(i < 100) {
      Serial.print(i);
      Serial.print(" outcome: ");
    //}

    if(wins[i] == LOSE) {
      gamesLost++;
      if(i < 100) {
        Serial.print("lose   ");
      }

    } else if(wins[i] == PERFECT){
      gamesWon++;
      gamesWarning++;
      gamesPerfect++;
      if(i < 100) {
        Serial.print("perfect");
      }
    } else if (wins[i] == WARNING) {
      gamesWon++;
      gamesWarning++;
      if(i < 100) {
        Serial.print("warning");
      }
    } else {
      gamesWon++;
      if(i < 100) {
        Serial.print("win    ");
      }
    }
    if(i < 100) {
      Serial.print(" ended at: ");
      Serial.print(scores[i]);
      Serial.print("   moves taken: ");
      for(int j = 0; j < 3; j++) {
        Serial.print(moves[i][j]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  Serial.print(NUM_SIMS);
  Serial.println(" games simulated");
  Serial.print(gamesLost);
  Serial.println(" games lost");
  Serial.print(gamesWon);
  Serial.println(" games won");
  Serial.print(gamesWarning);
  Serial.print(" games ended in the warning area (");
  Serial.print(gamesPerfect);
  Serial.print(" perfect, ");
  Serial.print(gamesWarning - gamesPerfect);
  Serial.println(" warning only)");
  Serial.print(gamesWon - gamesWarning);
  Serial.println(" games ended below the danger zone");
}

uint8_t moves[3];

void testOutcomes() {
  for(int i = 0; i < 100; i++) {
    generateLoseGame();
    if(getScore() < 25) {
      Serial.print("bad lose game: ");
      outputMoves();
    }
  }
  for(int i = 0; i < 100; i++) {
    generatePerfectGame();
    if(getScore() != PERFECT_SCORE) {
      Serial.print("bad perfect game: ");
      outputMoves();
    }
  }
  for(int i = 0; i < 100; i++) {
    generateWarningGame();
    if(getScore() < WARNING_POSITION || getScore() >= PERFECT_SCORE) {
      Serial.print("bad warning game: ");
      outputMoves();
    }
  }
  for(int i = 0; i < 100; i++) {
    generateEasyWin();
    if(getScore() >= WARNING_POSITION ) {
      Serial.print("bad easy win: ");
      outputMoves();
    }
  }
}

uint8_t getScore() {
  return moves[0] + moves[1] + moves[2];
}

void outputMoves() {
  Serial.print("total: ");
  Serial.print(getScore());
  Serial.print(" moves: ");
  Serial.print(moves[0]);
  Serial.print(" ");
  Serial.print(moves[1]);
  Serial.print(" ");
  Serial.println(moves[2]);
}


int preSelectedDistance() {
  if(turn == 0) {
    selectGameType();
  }
  int move = moves[turn];
  turn++;
  turn %= 3;
  return move;
}

int losePercent = 35;
int perfectPercent = 14;
int warningPercent = 35;
void selectGameType() {
  int gameSeed = random(0,100);
  if(gameSeed < losePercent) {
    generateLoseGame();
  } else if(gameSeed < perfectPercent + losePercent) {
    generatePerfectGame();
  } else if(gameSeed < perfectPercent + losePercent + warningPercent) {
    generateWarningGame();
  } else {
    generateEasyWin();
  }
}

void generateLoseGame() {
  uint8_t move1, move2, move3;
  do {
    move1 = random(1,13);
    move2 = random(1,13);
    move3 = random(1,13);
  } while(move1 + move2 + move3 < 25);
  moves[0] = move1;
  moves[1] = move2;
  moves[2] = move3;
}

void generateEasyWin() {
  uint8_t move1, move2, move3;
  do {
    move1 = random(1,13);
    move2 = random(1,13);
    move3 = random(1,13);
  } while(((move1 + move2 + move3) >= 21) || ((move1 + move2) < 10));
  moves[0] = move1;
  moves[1] = move2;
  moves[2] = move3;
}


void generateWarningGame() {
  uint8_t move1, move2, move3;
  do {
    move1 = random(1,13);
    move2 = random(1,13);
    move3 = random(1,13);
  } while(((move1 + move2 + move3) >= 24) ||
          ((move1 + move2 + move3) < 21) ||
          ((move1 + move2) < 10));
  moves[0] = move1;
  moves[1] = move2;
  moves[2] = move3;

}

void generatePerfectGame() {
  uint8_t move1, move2, move3;
  do {
    move1 = random(1,13);
    move2 = random(1,13);
    move3 = random(1,13);
  } while(((move1 + move2 + move3) != 24) ||
          ((move1 + move2) < 10));
  moves[0] = move1;
  moves[1] = move2;
  moves[2] = move3;
}


uint8_t generateGameMin12() {
  uint8_t move1, move2, move3;
  do {
    move1 = random(1,13);
    move2 = random(1,13);
  } while(move1 + move2 < 12);
  move3 = random(1,13);
  moves[0] = move1;
  moves[1] = move2;
  moves[2] = move3;
}


uint8_t randomDistanceMin12() {
  if(turn == 0) {
    generateGameMin12();
  }
  int move = moves[turn];
  turn++;
  turn %= 3;
  return move;
}

uint8_t randomDistance() {
  static uint8_t lastDistance = -1;
  uint8_t newDistance = -1;
  do {
    newDistance = random(1,13);
    //loop u    
  } while(newDistance == lastDistance);
  lastDistance = newDistance;
  return newDistance;
}

float invCDF(float uniformRandom) {
  // For linearly increasing weights, the CDF is a quadratic function.
  // Solving for its inverse gives us the following formula:
  return sqrt(144 * uniformRandom);
}

uint8_t randomDistanceWeighted() {
  static uint8_t lastDistance = -1;
  uint8_t newDistance = -1;
  do {
  float uniformRandom = random(10000) / 10000.0;
   float nonUniformRandom = invCDF(uniformRandom);

    // We'll round to the nearest integer to get an integer result
    newDistance = round(nonUniformRandom);
    if (newDistance == 0) {
      newDistance = 1;
    } else if (newDistance == 13) {
      newDistance = 12;
    }
  } while(newDistance == lastDistance);
  lastDistance = newDistance;
  return newDistance;
}
