import Assignment as ass


def main():
    easyboard = ass.create_sudoku_csp("easy.txt")
    easySsolution = easyboard.backtracking_search()
    print("easy solution")
    ass.print_sudoku_solution(easySsolution)
    print("number of times backtrack was called:", easyboard.backtracks)
    print("number of fails :", easyboard.failed)
    print()

    mediumBoard = ass.create_sudoku_csp("medium.txt")
    mediumSolution = mediumBoard.backtracking_search()
    print("medium solution")
    ass.print_sudoku_solution(mediumSolution)
    print("number of times backtrack was called:", mediumBoard.backtracks)
    print("number of fails :", mediumBoard.failed)
    print()

    hardboard = ass.create_sudoku_csp("hard.txt")
    hardSolution = hardboard.backtracking_search()
    print("hard solution")
    ass.print_sudoku_solution(hardSolution)
    print("number of times backtrack was called:", hardboard.backtracks)
    print("number of fails :", hardboard.failed)
    print()

    veryhardBoard = ass.create_sudoku_csp("veryhard.txt")
    veryhardSolution = veryhardBoard.backtracking_search()
    print("very hard solution")
    ass.print_sudoku_solution(veryhardSolution)
    print("number of times backtrack was called:", veryhardBoard.backtracks)
    print("number of fails :", veryhardBoard.failed)
    print()

if __name__ == "__main__":
    main()