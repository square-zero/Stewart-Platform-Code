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
    helv24 = tkFont.Font(family='Helvetica',
    size=24, weight='bold')
    frame = Frame(master)
    frame.grid(row=0, column=0)
    self.button = Button(frame, 
                         text="QUIT", fg="red", font="helv12",
                         command=frame.quit)
    #self.button.pack(side=LEFT, pady=70)
    self.button.grid(row=1, column=1, padx= 195, pady=70)
    self.bartender = Button(frame,
                         text="Deliver Drinks", font="helv12",
                         command=self.deliver_drinks)
    #self.bartender.pack(side=LEFT, pady=70)
    self.bartender.grid(row=1, column=2, pady=70)

    self.customer = Button(frame, text="Place Order", font="helv12",
			 command=self.place_order)
    #self.customer.pack(side=LEFT, pady=70)
    self.customer.grid(row=1, column=3, pady=70)


  def deliver_drinks(self):
    #bartender_count += 1
    print "Delivery in process"

  def place_order(self):
    helv24 = tkFont.Font(family='Helvetica',
    size=24, weight='bold')
    helv48 = tkFont.Font(family='Helvetica',
    size=48, weight='bold')
    top = Toplevel()
    top.title("Tipsy Engineers, Inc.")

    msg = Message(top, text="Drink Menu", font="helv48")
    msg.grid(row=0, column=0, padx=150, pady=100)
    top.wm_geometry("1200x920")

    msg = Message(top)

    drink1 = Button(top, text="Strawberry Lemonade", font="helv24",    command=top.destroy)
    drink1.grid(row=1, column=1, pady=15)

    drink2 = Button(top, text="Mojito", font="helv24", command=top.destroy)
    drink2.grid(row=2, column=1, pady=15)

    drink3 = Button(top, text="Coca-Cola", font="helv24", command=top.destroy)
    drink3.grid(row=3, column=1, pady=15)

    drink4 = Button(top, text="Sprite", font="helv24", command=top.destroy)
    drink4.grid(row=4, column=1, pady=15)

    drink5 = Button(top, text="Sparkling Cider", font="helv24", command=top.destroy)
    drink5.grid(row=5, column=1, pady=15)

    drink6 = Button(top, text="Shirley Temple", font="helv24", command=top.destroy)
    #drink1.pack(side=RIGHT)
    drink6.grid(row=1, column=2, pady=15)

    drink7 = Button(top, text="Roy Rogers", font="helv24", command=top.destroy)
    drink7.grid(row=2, column=2, pady=15)

    drink8 = Button(top, text="The DD", font="helv24", command=top.destroy)
    drink8.grid(row=3, column=2, pady=15)

    drink9 = Button(top, text="Root Beer Float", font="helv24", command=top.destroy)
    drink9.grid(row=4, column=2, pady=15)

    drink10 = Button(top, text="Club Soda and Lime", font="helv24", command=top.destroy)
    drink10.grid(row=5, column=2, pady=15)

    #customer_count +=1
    #window = Tk.Toplevel() #Check
    #window.wm_geometry("1200x920")
    #window.button = Button(frame2, 
                         #text="QUIT", fg="red", font="helv12",
                         #command=frame2.quit)
    #window.button.pack(side=LEFT, pady=70)
    print "Drinks will be delivered momentarily"

root = Tk.Tk()
root.title("Tipsy Engineers, Inc.")
root.wm_geometry("1200x920")
background_image=Tk.PhotoImage(file="martini.png")
background_label = Tk.Label(root, image=background_image)
#Place at x = 20 to center and y = 20 to give space between buttons and picture
background_label.place(x=20, y=30, relwidth=1, relheight=1)
app = App(root)
root.mainloop()
