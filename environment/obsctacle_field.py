from PIL import Image
import random
import matplotlib.pyplot as plt

def get_tetromino_pixels(position, tetromino_shape):

    x = position[0]
    y = position[1]

    if tetromino_shape == 'I':
        return [(x, y), (x, y+1), (x, y+2), (x, y+3)]
    
    if tetromino_shape == 'L':
        return [(x, y), (x+1, y), (x+1, y+1), (x+1, y+2)]
    
    if tetromino_shape == 'Step':
        return [(x, y), (x, y+1), (x+1, y+1), (x+1, y+2)]
    
    if tetromino_shape == 'T':
        return [(x, y+1), (x+1, y), (x+1, y+1), (x+1, y+2)]

def get_current_coverage(img, obstacle_color=(0, 0, 0)):
    
    obstacle_pixel_count = 0
    
    for pixel in img.getdata():
        if pixel == obstacle_color:
            obstacle_pixel_count+= 1
    
    return 100 * obstacle_pixel_count / (img.size[0] * img.size[1])

def put_obstacles_on_field(img, tetromino_pixels, obstacle_color=(0, 0, 0)):

    for position in tetromino_pixels:
        img.putpixel(position, obstacle_color)
    
    return img

def create_grid_with_obstacles(grid_size=128, desired_coverage=10):

    image_size = (grid_size, grid_size)
    obstacle_color = (0, 0, 0)

    # set a blank canvas for putting in the obstacles
    img = Image.new(mode='RGB', size=image_size, color=(255, 255, 255))
    
    current_coverage = 0
    count = 0
    while current_coverage < desired_coverage:
        count += 1
        
        # get random position for putting tetrominos
        pos_x = random.randint(0, image_size[0] - 4)
        pos_y = random.randint(0, image_size[0] - 4)

        # choose a tetromino at random
        tetromino_list = ['I', 'L', 'Step', 'T']
        tetromino_shape = random.choice(tetromino_list)

        # get pixels where obstacle color can be put
        tetromino_pixels = get_tetromino_pixels((pos_x, pos_y), tetromino_shape)

        # put obstacle color on to the pixel values for corresponding tetromino
        img = put_obstacles_on_field(img, tetromino_pixels, obstacle_color)

        # update the current coverage
        current_coverage = get_current_coverage(img, obstacle_color)
        
        # plt.imshow(img)

        # uncomment this part to see the process of obstacle generation
        # if (count % 50 == 0): 
        #     plt.pause(1e-6)

    
    print(f'\nObstacle field of {desired_coverage} percent coverage complete!\n')
    # plt.show()

    return img
    

# if __name__ == '__main__':
#     img = create_grid_with_obstacles()