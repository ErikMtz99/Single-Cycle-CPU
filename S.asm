# Maximum of an array:
# Input: -- (array)
# Output: s4 (maximum)

.data                 # directive indicating start of the data segment
.align  2             # set data alignment to 4 bytes

image1:                # 1st image, array of 4bytes
.word  0x5350412e,0x00000002,0x00000003,0x11223300,0x2200aaff,0x00ffff00,0x03565654,0x1b459748,0xecf39baa  

image2:                # 2nd image, array of 4bytes
.word  0x53504132,0x00000002,0x00000003,0x14524710,0x462cc5df,0xff0000ff,0x14ef1aa5,0x1b445748,0x1cf39b3a  

image_out:                # output image
.word  

.text                 # beginning of the text segment (or code segment)
start:

#la
#auipc rd, PC + image1[31:12]
#addi rd, rd, image1[11:0]
la   s1, image1        # store address of the "image1" to the register s1
la   s2, image2        # store address of the "image2" to the register s2
la   s3, image_out        # store address of the "image2" to the register s2

addi s4, zero, 0      # initialization instruction of for cycle: i=0, where i=s4
addi s5, zero, 9      # set the upper bound for cycle
addi s6, zero, 0      #initialize number of pixels of output image
addi s7, zero, 3      # s7 = 3
addi s10, zero, 1     #s10 = 1
addi s11, zero, 2     #s11 = 2

lui t4, 0x00fff       
addi s8, t4, 0xffffffff    #constant 0x00FFFFFF in s8 

lui t5, 0xFF000       
addi s9, t5, 0x0    #constant 0xFF000000 in s9

for:
  beq  s4, s5, done   # if s4 == s5, go to label done and break the cycle
  lw   t0, 0x0(s1)    # load value from image1 to t0
  lw   t1, 0x0(s2)    # load value from image1 to t1
  
  beq s4, x0, line_0   # if (i = 0) go to line_0, if not go to next line
  beq s4, s10, line_1_2 # if (i = 1) go to line_1_2, if not go to next line
  beq s4, s11, line_1_2 # if (i = 2) go to line_1_2, if not go to next line
  #usar slt para checar si es mayor o igual a 3 y que lo haga hasta que se acabe
  slt t3, s4, s7        #if s4 (i) < 3 => t = 1
  beq t3, x0, line_pixels #if t3 = 0, go to line_pixels

  skip:
  addi s1, s1, 0x4    # increment offset and move to the other value in the image1
  addi s2, s2, 0x4    # increment offset and move to the other value in the image2
  addi s3, s3, 0x4    # increment offset and move to the other value in the output image
  addi s4, s4, 0x1    # increment number of passes through the cycle (i++).
  jal x1, for	      #jump to **for** label

line_0:
addi t2, t1, 0x4    # save in temp2 the value of output img header
sw t2, 0x0(s3)      #save in first element of s3 the value of temp2  
jal x1, skip	      #jump to **skip** label
 
line_1_2:
sw s2, 0x0(s3)        #save the same width/heigth as image 2
jal x1, skip	      #jump to **skip** label

line_pixels:           #each pixel is 4 bytes (32 bits)
and s3, s3, s8         #clear first byte of output image
add s3, s3, s9         #set alpha chanel to 0xFF
add s3, s1, s2         #sum of color channels
jal x1, skip	      #jump to **skip** label
#addu.qb s3, s1, s2   
done:
