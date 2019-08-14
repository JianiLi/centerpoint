


def main(file):
    try:
        point_file = file
    except IndexError:
        point_file = None

    plt.ion()
    plt.show()







if __name__ == '__main__':
    # file = sys.argv[1]
    file ='/Users/jianili/Desktop/HamSandwichViz/pointsets/nointersection.txt'
    main(file)
