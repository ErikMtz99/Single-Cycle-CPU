# Maximum of an array:
# Input: -- (array)
# Output: s4 (maximum)

.data                 # directive indicating start of the data segment
.align  2             # set data alignment to 4 bytes

.text                 # beginning of the text segment (or code segment)
start:

lw   s1, 0x4(zero)    #save in s1 the pointer to image 1
lw   s2, 0x8(zero)    #save in s2 the pointer to image 2
lw   s3, 0xC(zero)    #save in s3 the pointer to output image

addi s4, zero, 0      # initialization instruction of for cycle: i=0, where i=s4
addi s5, zero, 9      # set the upper bound for cycle
addi s6, zero, 0      #initialize number of pixels of output image
addi s7, zero, 3      # s7 = 3
addi s10, zero, 1     #s10 = 1
addi s11, zero, 2     #s11 = 2 
addi a0, zero, 0      #initialize return value a0 to 0

lui t4, 0x01000       
addi s8, t4, -1    #constant 0x00FFFFFF in s8 

lui t5, 0xFF000       
addi s9, t5, 0x0    #constant 0xFF000000 in s9

addi t4, zero, 0     #initialize index for multiplication

for:
  beq  s4, s5, for_return   # if s4 == s5, go to label for_return and break the cycle
  lw   t0, 0x0(s1)    # load value saved in address 0x4 of memory to t0
  lw   t1, 0x0(s2)    # load value saved in address 0x8 of memory to t1
  
  beq s4, x0, line_0   # if (i = 0) go to line_0, if not go to next line
  beq s4, s10, line_1 # if (i = 1) go to line_1, if not go to next line
  beq s4, s11, line_2 # if (i = 2) go to line_2, if not go to next line
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
 
line_1:
add t5, x0, t0        #save in a3 the value of heigth
sw t0, 0x0(s3)        #save the same heigth as image 2
jal x1, skip	      #jump to **skip** label

line_2:
add t6, x0, t1        #save in t1 the value of heigth
sw t1, 0x0(s3)        #save the same width as image 2
jal x1, skip	      #jump to **skip** label

line_pixels:           #each pixel is 4 bytes (32 bits)
lw  a4, 0x0(s3)        # load value from output image to a4
and a4, a4, s8         #clear first byte of output image
add a4, a4, s9         #set alpha chanel to 0xFF
add a4, t0, t1         #sum of color channels
sw a4, 0x0(s3)        #save the value in output image
jal x1, skip	      #jump to **skip** label
#addu.qb s3, s1, s2   

for_return:
  beq  t4, t5, done    # if t4 == heigth, go to label done and break the cycle
  add a0, a0, t6       #save in return value register a0 the number of pixels
  sw a0, 0x10(zero)       #save a0 value in 0x10 address    
  addi t4, t4, 0x1    # increment number of passes through the cycle (i++).
  jal x1, for_return    #jump to **fo_return** label  

done:
