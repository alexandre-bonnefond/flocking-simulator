
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

def fileToCBP(filename) :

    f = open(filename, "r")
    lines = f.readlines()

    CBP = list()
    matrix = list()
    index = int(lines[0].strip())
    nextIsIndex = False
    for line in lines[1:] :
        if line == "\n" :
            nextIsIndex = True
            continue

        if nextIsIndex :
            index = int(line.strip())
            CBP.append(list(matrix))
            matrix = list()
            nextIsIndex = False
        else :
            matrix.append(line.strip().split(" "))
            
    for matrix in CBP :
        for li in range(len(matrix)) :
            line = matrix[li]
            tmp = list()
            nbOfValues = 4
            for i in range(0, len(line)-(nbOfValues-1), nbOfValues) :
                tmp.append((float(line[i]), float(line[i+1]), float(line[i+2]), float(line[i+3])))
            matrix[li] = tmp
    
    return CBP

def mergeCool(CBP) :
    height = len(CBP[0])
    width = len(CBP[0][0])
    agentCount = len(CBP)

    mean = np.zeros((width, height), np.float64)
    for y in range(height) :
        for x in range(width) :
            count = 0.0
            sampleCount = 0.0

            for agent in range(agentCount) :
                count += CBP[agent][y][x][0]
                sampleCount += CBP[agent][y][x][0] + CBP[agent][y][x][1]

            if sampleCount == 0.0 :
                mean[y][x] = 0.0
            else :
                mean[y][x] = count / sampleCount

            if mean[y][x] != 1.0 and mean[y][x] != 0.0 :
                print(mean)

    return mean


def oddsMerge(CBP) :
    P_mat = [ [1] * len(CBP[0][0]) ] * len(CBP[0])
    for agent in range(len(CBP)) :
        for y in range(len(CBP[agent])) :
            for x in range(len(CBP[agent][y])) :
                total = CBP[agent][y][x][0] + CBP[agent][y][x][1]
                if total == 0 :
                    continue

                P_occ = CBP[agent][y][x][0] / total # P_occ
                if P_occ == 1 : P_occ = .999999999999
                P_mat[y][x] *= P_occ / (1 - P_occ) # odds product
        
        for y in range(len(CBP[agent])) :
            for x in range(len(CBP[agent][y])) :
                P_mat[y][x] = P_mat[y][x] / (1 + P_mat[y][x]) # P_occ(x,y)

    return P_mat


def getAgent(CBP, index) : 
    
    height = len(CBP[0])
    width = len(CBP[0][0])

    data = np.zeros((width, height), np.float64)

    for y in range(len(CBP[index])) :
        for x in range(len(CBP[index][y])) :
            data[y][x] = CBP[index][y][x][0]

    return data


def display(CBP) :
    data = np.array(CBP)

    fig, ax = plt.subplots()
    im = ax.imshow(data, cmap="gray", vmin=0.0, vmax=1.0)

    # Loop over data dimensions and create text annotations.
    # for i in range(len(data)):
    #     for j in range(len(data[i])):
    #         if data[i, j] > 0.5 :
    #             col = "black"
    #         else :
    #             col = "w"
    # 
    #         text = ax.text(j, i, data[i, j],
    #                     ha="center", va="center", color=col)

    ax.set_title("Agents' data merged into a single global map")
    fig.tight_layout()

    return fig, ax


if __name__ == "__main__" :

    filename = "cbp_matrices_temp_forest_3000"
    CBP = fileToCBP(filename + ".dat")
    f = open("test_output.txt", "w")
    f.write( '\n'.join([ ' '.join([str(CBP[0][y][x][0]) for x in range(len(CBP[0][0]))]) for y in range(len(CBP[0]))]) )
    f.close()

    # print(CBP)
    merged = mergeCool(CBP)
    # print(merged)
    
    f = open("merge_output.txt", "w")
    f.write( '\n'.join([ ' '.join([str(merged[y][x]) for x in range(len(merged[0]))]) for y in range(len(merged))]) )
    f.close()

    fig, ax = display(merged)
    fig.savefig(fname="./merged_map_" + filename + ".png", bbox_inches='tight')
    plt.imsave("./merged_map_" + filename + "_imsave.png", merged, cmap="gray", vmin=0.0, vmax=1.0)
    plt.show()