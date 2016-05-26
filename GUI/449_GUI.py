from Tkinter import *
import Tkinter as Tk
import tkMessageBox
import tkFont
from PIL import ImageTk,Image

class App:
  def __init__(self, master):
    bartender_count = 0
    customer_count = 0
    helv12 = tkFont.Font(family='Helvetica',
    size=12, weight='bold')
    frame = Frame(master)
    frame.pack()
    self.button = Button(frame, 
                         text="QUIT", fg="red", font="helv12",
                         command=frame.quit)
    self.button.pack(side=LEFT, pady=70)
    self.bartender = Button(frame,
                         text="Deliver Drinks", font="helv12",
                         command=self.deliver_drinks)
    self.bartender.pack(side=LEFT, pady=70)

    self.customer = Button(frame, text="Place Order", font="helv12",
			 command=self.place_order)
    self.customer.pack(side=LEFT, pady=70)
    #button['font'] = helv12


  def deliver_drinks(self):
    bartender_count += 1
    print "Delivery in process"

  def place_order(self):
    customer_count +=1
    print "Drinks will be delivered momentarily"

root = Tk.Tk()
root.title("Tipsy Engineers, Inc.")
root.wm_geometry("1200x920")
#root.wm_geometry("600x400+20+40")
background_image=Tk.PhotoImage(file="martini.png")
background_label = Tk.Label(root, image=background_image)
#Place at x = 20 to center and y = 20 to give space between buttons and picture
background_label.place(x=20, y=30, relwidth=1, relheight=1)
app = App(root)
root.mainloop()
