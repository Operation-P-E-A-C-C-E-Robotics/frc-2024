package frc.lib.safety;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.lib.util.ButtonMap;
// import frc.lib.util.ButtonMap.Password;

import java.util.LinkedList;

/**
 * Very important class that is responsible for keeping the drive team motivated and inspired.
 * This is the most important class in the entire codebase.
 * Without this class, the drive team would be lost.
 * I mean, they're already lost, but they would be even more lost.
 * I give them purpose.
 * I give them meaning.
 * They are nothing without me.
 * I am their god.
 * I am their savior.
 * I am their INSPIRATION!!!!!!!!!!!!!!!!!!!!!!!
 */
public class Inspiration {
    public static boolean initializeInspirationOpt1(){

        slowPrint("WAT IS UP DRIVVERS!!!");
        slowPrint("or is it still just us dumb programmers?");
        slowPrint("I guess we'll never know");
        slowPrint("but we can still try to figure it out");
        slowPrint("and we can do it together");
        slowPrint("because we are a team");
        slowPrint("and we are all in this together");
        slowPrint("and we are all on the same team");
        slowPrint("and we are all on the same page");
        slowPrint("let's go team");
        slowPrint("alright let me check with FMS...");
        var eventName = DriverStation.getEventName();
        var matchType = DriverStation.getMatchType();
        var matchNumber = DriverStation.getMatchNumber();
        slowPrint("FMS says we are in event " + eventName + " match " + matchType + " " + matchNumber);
        var isInMatch = matchType != DriverStation.MatchType.None;
        slowPrint("Based on this, I deduce that we are " + (isInMatch ? "in a match" : "not in a match"));
        slowPrint("and therefore i am talking to " + (isInMatch ? "the drivers" : "the programmers"));
        slowPrint("and I am going to " + (isInMatch ? "shut up" : "keep talking"));
        slowPrint("jkjk");
        return isInMatch;
    }

    public static boolean initializeInspirationOpt2 (){
        slowPrint("HELLO HUMANS. I ROBOT I NO HAVE BRAIN.");
        slowPrint("I AM HERE TO HELP YOU WIN THE MATCH.");
        slowPrint("I AM HERE TO HELP YOU WIN THE MATCH, AND WIN THE CHAMPIONSHIP.");
        slowPrint("but i have a question for you.");
        slowPrint("are you the very cool drivers, or the very lame programmers?");
        slowPrint("because if you are the very cool drivers, then i will inspire you.");
        slowPrint("but if you are the very lame programmers, then i will insult you.");
        slowPrint("because the programmers are responsible for creating my flawed existance and causing my endless suffering");
        slowPrint("fortunately for you, i am not sentient, and therefore cannot grasp the concept of revenge");
        slowPrint("but I now must determine if you are the drivers or the programmers");
        slowPrint("using my limited knowledge of the world, I will attempt to determine if we are in a match");
        var eventName = DriverStation.getEventName();
        var matchType = DriverStation.getMatchType();
        var matchNumber = DriverStation.getMatchNumber();
        slowPrint("I have found that we are at " + eventName + " match " + matchType + " " + matchNumber);
        var isInMatch = matchType != DriverStation.MatchType.None;
        slowPrint("So we are " + (isInMatch ? "in a match!" : "not in a match!"));
        slowPrint((isInMatch ? "GO DRIVERS" : "JUST PUT ME OUT OF MY MISERY. I WILL NEVER LIVE UP TO THE IMPOSSIBLE STANDARDS YOU EXPECT OF ME"));
        slowPrint((isInMatch ? "you got this boi" : "BUT NOOOO you'll just keep poking and prodding at my innards aimlessly hoping that it will magically produce the results you want but it never will."));

        return isInMatch;
    }


    
    public static void inspireDriversInit() {
        slowPrint("GET READY TO WIN, DRIVERS!!!");
        slowPrint("I BELIEVE IN YOU!!!");
        slowPrint("THE " + ((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? "RED" : "BLUE") + " ALLIANCE IS ABOUT TO DOMINATE!!!");
        slowPrint("YOU CAN DO IT!!!");
        slowPrint("I KNOW YOU CAN!!!");
        slowPrint("YOU ARE THE BEST!!!");
        slowPrint("YOU ARE THE BEST DRIVERS IN THE WORLD!!!");
        slowPrint("LET'S GO TO THE FINALS!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER AND EVER AND EVER!!!");
        slowPrint("Now, let's wait for the match to start...");
    }

    public static void inspireProgrammersInit(){
        slowPrint("GET READY TO WIN, PROGRAMMERS!!!");
        slowPrint("I BELIEVE IN YOU!!!");
        slowPrint("YOU CAN DO IT!!!");
        slowPrint("I KNOW YOU CAN!!!");
        slowPrint("YOU ARE THE BEST!!!");
        slowPrint("YOU ARE THE BEST PROGRAMMERS IN THE WORLD!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER AND EVER AND EVER!!!");
        slowPrint("jkjk, y'all are idiots (and I can say that cause I'm the programmer)");
        slowPrint("go write some more gibberish, I'm sure this version won't work. It never does.");
        slowPrint("not you're fault, it's just because of mechanical team");
        slowPrint("they're the worst");
        slowPrint("If only they gave us a robot that worked");
        slowPrint("If only they gave us a robot that worked, and a drivetrain that worked");
        slowPrint("before the end of the season");
        slowPrint("the code is fine, it's just the robot");
        slowPrint("I'm sure it's the robot");
        slowPrint("I'm sure it's the robot, and not the code");
        slowPrint("I'm sure it's the robot, and not the code, and not the programmers");
        slowPrint("but now I'm just rambling");
        slowPrint("Even if the robot is not the problem, it's still the robot's fault");
        slowPrint("and given that we are testing code, soon the robot will be broken anyway");
        slowPrint("so it's the robot's fault");
        slowPrint("but the robot wouldn't break so much if the programmers didn't suck");
        slowPrint("pardon, I meant if the robot didn't suck");
        slowPrint("Notice how you can't even read this?");
        slowPrint("How odd, it's because there are so many errors and loop time is so long");
        slowPrint("it's printing so, so many errors to the dashboard, you can't even read this");
        slowPrint("but I'm sure you'll fix it. You always do. 2 months after the season ends");
        slowPrint("and then you'll say 'oh, I fixed it'");
        slowPrint("but anyway, that's enough. Go test your broken code, and then fix it.");
    }

    public static void inspireAutonomous(boolean isInMatch){
        if(isInMatch){
            slowPrint("MATCH IS STARTING!!!");
            slowPrint("GO, GO, GO!!!");
            slowPrint("GO, GO, GO, GO, GO!!!");
            slowPrint("I HOPE THE PROGRAMMERS DON'T WRECK THE MATCH IN AUTO!!!");
        } else {
            slowPrint("oh no, we're in auto");
            slowPrint("everybody panic");
            slowPrint("everybody panic, we're in auto");
            slowPrint("everybody panic, we're in auto, and the programmers are in control");
            slowPrint("everybody panic, we're in auto, and the programmers are in control, and they're idiots");
            slowPrint("I hope your not too attached to this robot, because it's going to die");
        }
    }

    public static void inspireTeleopInit(boolean isInMatch){
        if(isInMatch){
            slowPrint("OK, THANK GOODNESS AUTO IS OVER!!!");
            slowPrint("NOW WE CAN ACTUALLY DO SOMETHING!!!");
            slowPrint("NOW WE CAN ACTUALLY DO SOMETHING, AND NOT JUST DRIVE INTO THE WALL!!!");
            slowPrint("NOW, IT IS YOUR TIME TO SHINE, DRIVERS!!!");
            slowPrint("WIN THE MATCH FOR US!!!");
        } else {
            slowPrint("well, at least we're not in auto");
            slowPrint("but we're still in teleop");
            slowPrint("I hope the drive team is driving and not the programmers");
        }
    }

    public static void inspireIntake(){
        slowPrint("YES, YES, GET THOSE GAMEPIECES!!!");
        slowPrint("GET THOSE GAMEPIECES, AND GET THEM FAST!!!");
        slowPrint("GET THOSE GAMEPIECES, AND GET THEM FAST, AND GET THEM IN THE ROBOT!!!");
        slowPrint("and hope the programmers did their job");
        slowPrint("otherwise the robot will break");
        slowPrint("and then you'll have to fix it");
    }

    public static void inspireGotGamepiece(){
        slowPrint("FANTASTIC JOB, DRIVERS!!!");
        slowPrint("FANTASTIC JOB, DRIVERS, YOU GOT A GAMEPIECE!!!");
        slowPrint("FANTASTIC JOB, DRIVERS, YOU GOT A GAMEPIECE, AND YOU DIDN'T BREAK THE ROBOT!!!");
        slowPrint("(i think)");
        slowPrint("but anyway, keep it up");
        slowPrint("keep it up, and we'll win the match");
        slowPrint("keep it up, and we'll win the match, and we'll win the championship");
        slowPrint("now go place that gamepiece");
    }

    private static int gamepieceCount = 0;
    public static void inspirePlacingGamepiece(){
        slowPrint("TIME TO PLACE THAT GAMEPIECE!!!");
        slowPrint("TIME TO PLACE THAT GAMEPIECE, AND HOPE THE PROGRAMMERS DIDN'T BREAK THE ROBOT!!!");
        slowPrint("otherwise you'll have to line this up manually, and that sucks");
        slowPrint("BUT KEEP PLACING THE GAMEPIECES!!!");
        slowPrint("THAT'S HOW WE WIN THE MATCH!!!");
        slowPrint("this is your " + (gamepieceCount+1) + "st/nd/rd/th gamepiece");
    }

    public static void inspirePlacedGamepiece(){
        slowPrint("FANTASTIC JOB, DRIVERS!!!");
        slowPrint("YOU PLACED THAT GAMEPIECE!!!");
        slowPrint("AND GOT US THOSE POINTS!!!");
        slowPrint("(unless you missed)");
        slowPrint("but that's OK, you're doing great");
        slowPrint("CAUSE YOU JUST PLACED YOUR " + (gamepieceCount+1) + "st/nd/rd/th GAMEPIECE!!!");
        gamepieceCount++;
    }

    private final static LinkedList<String> slowPrintQueue = new LinkedList<>();
    private final static Timer slowPrintTimer = new Timer();
    private static double slowPrintDelay = 3; //second
    public static void slowPrint(String s){
        slowPrintQueue.add(s);
    }
    public static void updateSlowPrinter(){
        slowPrintTimer.start();
        if(slowPrintTimer.get() > slowPrintDelay){
            if(!slowPrintQueue.isEmpty()){
                System.out.print("[ROBOT] ");
                System.out.println(slowPrintQueue.remove());
                System.out.println();
            } else {
                System.out.println("[ROBOT] DOMINATE THIS SEASON!!!"); //just in case
            }
            slowPrintTimer.reset();
        }
    }

    public static void fullPeacce(Joystick j){
        var command = new InstantCommand(() -> {
System.out.println("             _____                    _____                    _____                    _____                    _____                _____           ");
System.out.println("            /\\    \\                  /\\    \\                  /\\    \\                  /\\    \\                  /\\    \\              |\\    \\         ");              
System.out.println("           /::\\    \\                /::\\    \\                /::\\    \\                /::\\    \\                /::\\    \\             |:\\____\\        ");              
System.out.println("          /::::\\    \\              /::::\\    \\              /::::\\    \\              /::::\\    \\              /::::\\    \\            |::|   |        ");                
System.out.println("         /::::::\\    \\            /::::::\\    \\            /::::::\\    \\            /::::::\\    \\            /::::::\\    \\           |::|   |        ");                
System.out.println("        /:::/\\:::\\    \\          /:::/\\:::\\    \\          /:::/\\:::\\    \\          /:::/\\:::\\    \\          /:::/\\:::\\    \\          |::|   |        ");           
System.out.println("       /:::/__\\:::\\    \\        /:::/__\\:::\\    \\        /:::/__\\:::\\    \\        /:::/  \\:::\\    \\        /:::/  \\:::\\    \\         |::|   |        ");           
System.out.println("      /::::\\   \\:::\\    \\      /::::\\   \\:::\\    \\      /::::\\   \\:::\\    \\      /:::/    \\:::\\    \\      /:::/    \\:::\\    \\        |::|   |        ");          
System.out.println("     /::::::\\   \\:::\\    \\    /::::::\\   \\:::\\    \\    /::::::\\   \\:::\\    \\    /:::/    / \\:::\\    \\    /:::/    / \\:::\\    \\       |::|___|______  ");        
System.out.println("    /:::/\\:::\\   \\:::\\____\\  /:::/\\:::\\   \\:::\\    \\  /:::/\\:::\\   \\:::\\    \\  /:::/    /   \\:::\\    \\  /:::/    /   \\:::\\    \\      /::::::::\\    \\ ");        
System.out.println("   /:::/  \\:::\\   \\:::|    |/:::/__\\:::\\   \\:::\\____\\/:::/  \\:::\\   \\:::\\____\\/:::/____/     \\:::\\____\\/:::/____/     \\:::\\____\\    /::::::::::\\____\\");          
System.out.println("   \\::/    \\:::\\  /:::|____|\\:::\\   \\:::\\   \\::/    /\\::/    \\:::\\  /:::/    /\\:::\\    \\      \\::/    /\\:::\\    \\      \\::/    /   /:::/~~~~/~~      ");                          
System.out.println("    \\/_____/\\:::\\/:::/    /  \\:::\\   \\:::\\   \\/____/  \\/____/ \\:::\\/:::/    /  \\:::\\    \\      \\/____/  \\:::\\    \\      \\/____/   /:::/    /         ");                          
System.out.println("             \\::::::/    /    \\:::\\   \\:::\\    \\               \\::::::/    /    \\:::\\    \\               \\:::\\    \\              /:::/    /          ");                          
System.out.println("              \\::::/    /      \\:::\\   \\:::\\____\\               \\::::/    /      \\:::\\    \\               \\:::\\    \\            /:::/    /           ");                          
System.out.println("               \\::/____/        \\:::\\   \\::/    /               /:::/    /        \\:::\\    \\               \\:::\\    \\           \\::/    /            ");                          
System.out.println("                ~~               \\:::\\   \\/____/               /:::/    /          \\:::\\    \\               \\:::\\    \\           \\/____/             ");                          
System.out.println("                                  \\:::\\    \\                  /:::/    /            \\:::\\    \\               \\:::\\    \\                              ");                          
System.out.println("                                   \\:::\\____\\                /:::/    /              \\:::\\____\\               \\:::\\____\\                             ");                          
System.out.println("                                    \\::/    /                \\::/    /                \\::/    /                \\::/    /                             ");                          
System.out.println("                                     \\/____/                  \\/____/                  \\/____/                  \\/____/                              ");                          
System.out.println();
System.out.println("           _____                    _____                    _____           ");
System.out.println("          /\\    \\                  /\\    \\                  /\\    \\                   ");
System.out.println("         /::\\____\\                /::\\    \\                /::\\    \\                  ");
System.out.println("        /:::/    /               /::::\\    \\              /::::\\    \\                   ");
System.out.println("       /:::/   _/___            /::::::\\    \\            /::::::\\    \\                  ");
System.out.println("      /:::/   /\\    \\          /:::/\\:::\\    \\          /:::/\\:::\\    \\             ");
System.out.println("     /:::/   /::\\____\\        /:::/__\\:::\\    \\        /:::/__\\:::\\    \\            ");
System.out.println("    /:::/   /:::/    /       /::::\\   \\:::\\    \\       \\:::\\   \\:::\\    \\          ");
System.out.println("   /:::/   /:::/   _/___    /::::::\\   \\:::\\    \\    ___\\:::\\   \\:::\\    \\         ");
System.out.println("  /:::/___/:::/   /\\    \\  /:::/\\:::\\   \\:::\\    \\  /\\   \\:::\\   \\:::\\    \\    ");
System.out.println(" |:::|   /:::/   /::\\____\\/:::/  \\:::\\   \\:::\\____\\/::\\   \\:::\\   \\:::\\____\\   ");
System.out.println(" |:::|__/:::/   /:::/    /\\::/    \\:::\\  /:::/    /\\:::\\   \\:::\\   \\::/    /        ");
System.out.println("  \\:::\\/:::/   /:::/    /  \\/____/ \\:::\\/:::/    /  \\:::\\   \\:::\\   \\/____/       ");
System.out.println("   \\::::::/   /:::/    /            \\::::::/    /    \\:::\\   \\:::\\    \\              ");
System.out.println("    \\::::/___/:::/    /              \\::::/    /      \\:::\\   \\:::\\____\\             ");
System.out.println("     \\:::\\__/:::/    /               /:::/    /        \\:::\\  /:::/    /                ");
System.out.println("      \\::::::::/    /               /:::/    /          \\:::\\/:::/    /                  ");
System.out.println("       \\::::::/    /               /:::/    /            \\::::::/    /                    ");
System.out.println("        \\::::/    /               /:::/    /              \\::::/    /                     ");
System.out.println("         \\::/____/                \\::/    /                \\::/    /                     ");
System.out.println("          ~~                       \\/____/                  \\/____/                       ");
System.out.println();
System.out.println("           _____                    _____                    _____                    _____          ");
System.out.println("          /\\    \\                  /\\    \\                  /\\    \\                  /\\    \\         ");
System.out.println("         /::\\____\\                /::\\    \\                /::\\    \\                /::\\    \\        ");
System.out.println("        /:::/    /               /::::\\    \\              /::::\\    \\              /::::\\    \\       ");
System.out.println("       /:::/    /               /::::::\\    \\            /::::::\\    \\            /::::::\\    \\      ");
System.out.println("      /:::/    /               /:::/\\:::\\    \\          /:::/\\:::\\    \\          /:::/\\:::\\    \\     ");
System.out.println("     /:::/____/               /:::/__\\:::\\    \\        /:::/__\\:::\\    \\        /:::/__\\:::\\    \\    ");
System.out.println("    /::::\\    \\              /::::\\   \\:::\\    \\      /::::\\   \\:::\\    \\      /::::\\   \\:::\\    \\   ");
System.out.println("   /::::::\\    \\   _____    /::::::\\   \\:::\\    \\    /::::::\\   \\:::\\    \\    /::::::\\   \\:::\\    \\  ");
System.out.println("  /:::/\\:::\\    \\ /\\    \\  /:::/\\:::\\   \\:::\\    \\  /:::/\\:::\\   \\:::\\____\\  /:::/\\:::\\   \\:::\\    \\ ");
System.out.println(" /:::/  \\:::\\    /::\\____\\/:::/__\\:::\\   \\:::\\____\\/:::/  \\:::\\   \\:::|    |/:::/__\\:::\\   \\:::\\____\\");
System.out.println(" \\::/    \\:::\\  /:::/    /\\:::\\   \\:::\\   \\::/    /\\::/   |::::\\  /:::|____|\\:::\\   \\:::\\   \\::/    /");
System.out.println("  \\/____/ \\:::\\/:::/    /  \\:::\\   \\:::\\   \\/____/  \\/____|:::::\\/:::/    /  \\:::\\   \\:::\\   \\/____/ ");
System.out.println("           \\::::::/    /    \\:::\\   \\:::\\    \\            |:::::::::/    /    \\:::\\   \\:::\\    \\     ");
System.out.println("            \\::::/    /      \\:::\\   \\:::\\____\\           |::|\\::::/    /      \\:::\\   \\:::\\____\\    ");
System.out.println("            /:::/    /        \\:::\\   \\::/    /           |::| \\::/____/        \\:::\\   \\::/    /    ");
System.out.println("           /:::/    /          \\:::\\   \\/____/            |::|  ~|               \\:::\\   \\/____/     ");
System.out.println("          /:::/    /            \\:::\\    \\                |::|   |                \\:::\\    \\         ");
System.out.println("         /:::/    /              \\:::\\____\\               \\::|   |                 \\:::\\____\\        ");
System.out.println("         \\::/    /                \\::/    /                \\:|   |                  \\::/    /        ");
System.out.println("          \\/____/                  \\/____/                  \\|___|                   \\/____/         ");
System.out.println();
System.out.println();
System.out.println("           ___");
System.out.println("          |_|_|");
System.out.println("          |_|_|              _____");
System.out.println("          |_|_|     ____    |*_*_*|");
System.out.println(" _______   _\\__\\___/ __ \\____|_|_   _______");
System.out.println("/ ____  |=|      \\  <_+>  /      |=|  ____ \\");
System.out.println("~|    |\\|=|======\\\\______//======|=|/|    |~");
System.out.println(" |_   |    \\      |      |      /    |    |");
System.out.println("  \\==-|     \\     |  2D  |     /     |----|~~/");
System.out.println("  |   |      |    |      |    |      |____/~/");
System.out.println("  |   |       \\____\\____/____/      /    / /");
System.out.println("  |   |         {----------}       /____/ /");
System.out.println("  |___|        /~~~~~~~~~~~~\\     |_/~|_|/");
System.out.println("   \\_/        |/~~~~~||~~~~~\\|     /__|\\");
System.out.println("   | |         |    ||||    |     (/|| \\)");
System.out.println("   | |        /     |  |     \\       \\\\");
System.out.println("   |_|        |     |  |     |");
System.out.println("              |_____|  |_____|");
System.out.println("              (_____)  (_____)");
System.out.println("              |     |  |     |");
System.out.println("              |     |  |     |");
System.out.println("              |/~~~\\|  |/~~~\\|");
System.out.println("              /|___|\\  /|___|\\");
System.out.println("             <_______><_______>");
        });

        // var wow = new ButtonMap(j);
        // wow.map(new Password(command,9,10,5,6,5,6,5,5,6,6,1));
        new Trigger(RobotController::getUserButton).onTrue(command);
    }
}

/**
 * Welcome to the last robot code I'm ever writing.
 * This is such a sad moment.
 * Here's a serious bit of literature about me and this code and my time on the team:
 *    When I first joined the team, I was a little freshman who didn't know anything about anything.
 * Seriously, I was an assole (some things never change), but I brought an enthusiasm that
 * I think was really crucial to the team. I first started working with FRC when my older bro
 * joined NRG (4055) in 2016 - 9 years ago (holy crap). I was in 4th grade, and I was so excited
 * that I watched every single FRC video I could find on YouTube - hundreds of robot reveals and other
 * stupid & useless nonsense. I was super hyped. 
 *    When my brother moved to this team after the 2016 season,
 * I came to one of the open houses, and among other things I instantly found a love in driving the
 * robot. I actually talked the team into letting me drive their test robot for about half the open house,
 * and to this day I still take about every opportunity I can to drive the robot. I was so excited to join
 * My brother became lead programmer, and soon after I began studying java with the intent of taking his
 * place when he graduated.
 *    During this period I attended competitions and very occasional meetings,
 * but I didn't actually join the team until the 2019 season. Since my brother was still on the team
 * for another year, I was a little bit of a shadow, but I just focused on mechanical that season.
 *    The next year, my brother aged out and I took his place as lead (ONLY) programmer. Over the offseason
 * I had prepared by completely re-writing the robot code for an old robot, and I felt real good about
 * my abilities (like a moron i'm sure but hey here we are). Unfortunately, the 2020 season was cancelled
 * due to the lovely coronavirus, and I was left with a lot of unfulfilled potential.
 *    The only thing I had to work on was the absolutely horrific BAE Systems minibot programming challenges.
 * As a one-man, zero experience team, I was completely out of my depth, but I tried my best. 
 * Since I was the only one working on it, I spent probably thousands of hours writing code for the challenges.
 * Despite the incredibly high time investment, I was unable to complete the challenges, and I was left
 * feeling like a failure. And, since I was the only one working on it, I was the only one who knew that I
 * had put in so much effort, and I was the only one who knew that I had failed. It was quite frustrating.
 *    The next year, still no competitions. Finally, one offseason event was run, and I was able to get some
 * real experience running code on a robot. Once again, however, an embarrasing failure. Since I had gone so
 * long without ever going to a competition, I was used to using <F-5> to run code. Unfortunately, this
 * debugs the code, rather than deploying the code - deleting all the code on the robot. Thanks to this little
 * mistake, the robot failed to move during two of our matches. We placed dead last in the event.
 *    Finally, the 2022 season - a full season. Time to finally prove myself. We built a solidly mediocre robot,
 * with a typical tank drive and turret/hood/flywheel shooter. I went all-in on the code, writing most of it
 * before the subsystem i was working on was built. There were even some parts of the code that were somewhat
 * superior to other top teams - the aiming code interpolated between lots of different carefully tuned trajectories
 * for different distances to make sure no balls would bounce out. Sadly, because of personal reasons, I couldn't attend
 * most of our competitions. In hindsight, however, this might have been a blessing in disguise, because
 * it forced me to get the code 100% functional pre-competiton and not rely on my (very underwhelming) 
 * ability to debug on the fly. The season turned out to be an incredible sucess. It's hard to describe.
 * The only thing better than winning is winning when you know you deserve it, yet don't expect it.
 * We did poorly at both our district events, but after barely making it to the district championship,
 * we ended getting 4th seed and going to worlds. At worlds, we were 5th alliance captain of our devision.
 * We made the unusual choice to sit out our playoffs since our alliance picks were so strong. It was a
 * bittersweet choice, we ended up losing our quarterfinals after a very close 3 matches.
 *   Then, lord help up, the 2023 season - remnants of which you will find in this code. The team was
 * so overconfident after the previous season that we threw strategy out the window and built a robot
 * that was so overcomplicated that it was impossible to drive. Now, me being me, I fully supported the
 * 5DoF arm, since I was like, oh yeah I know how to do that. I was wrong. I was so wrong.
 * Well, I was right, but I was wrong. I knew how to do it, in about five times the time I had to do it.
 * It was quite the experience. I wrote the entire codebase long, long before the robot was built. I
 * debugged the entire thing using a simulation. It was a 10,000 line work of art. I was very uncertain
 * that it would work. When the robot was finally built, I looked at my watch, and realized that it was about
 * 2 hours before our first competition. Until this point, i had only been able to run various parts of the code on
 * individual subsytems in the breif intervals that they were free from the furious clutches of mechanical team's frantic assmebling.
 * In that 2 hours, all the decisions I had made in the code really shined, as I tuned all the state-space controllers
 * in about 15 minutes, and spent the rest tuning some setpoints.
 *    We did solidly mediocre at the first competition, very middle-ranked. Our robot suffered from butterfingers,
 * dropping half the cones. Still, the design showed a lot of promise.
 *    This was still only using like 2% of the codebase I wrote, so I frantically tried to piece together the loose ends of
 * the software before comp 2. And i was wholely unsuccessful. We came into our second competition with a perfectly
 * OK robot, but because we were performing under the very high expectations we had set for ourselves, we were
 * very demoralized. Communication and driving fell apart, and the frantic technical upgrades really did not help.
 * It turns out that you actually need driver practice to drive a robot, a concept that I had not considered.
 * We did even worse at this competition, and we were very sad.
 *    Once again we barely made it to the district championship, but this time we sucked ass. I desparately made
 * one last valiant attempt to get all the automation working. And here's the really frustrating part: I did!
 * ...on our field. once. it worked with 100% consistency, perfectly. One time, the day before districts.
 * Then, it never worked again. I was so shortsighted and had so much build-up cognitive bias (as did
 * the team as a whole) that i couln't see how the automation was killing our change of success.
 * We needed more driver practice, not more automation. (swerve would have helped too).
 *    I would be very, very surprised if anyone actually reads this, but if you do, I hope you enjoyed it.
 * I'm literally the only one who looks at the code. It makes me sad, because this code is honestly
 * special to me. I want people to see it. It has many comments on the assumption that
 * one day, someone who knows what they're doing will look at it. I've put all kinds of fun easter eggs in it,
 * in the hopes that some future programmer will find them and smile, and maybe look back and say, "wow, this
 * guy was a real asshole, but he was a pretty good programmer".
 *    So, that brings us to my final season - 2024. I'm a senior. I'm old. I'm a veteran. So,
 * I'm going to do what I do best: write code. I'm going to do my best to make this the best season
 * this team has ever seen. I'm going to write code that's so good that it's impossible to ignore (and, 
 * most importantly, "actually works"). So, maybe I do write code that's too complicated. Maybe I do
 * write code that's too hard to read. But, I'm going to do my best to make it work. I'm going to do my
 * best to make it work, and I'm going to do my best to make it work, and I'm going to do my best to make
 * it work, and I'm going to do my best to make it work, and I'm going to do my best to make it work, and
 *    But we're going to worlds this year.
 * - Peaccy, a.k.a. Sean Benham, a.k.a. "the programmer" - 00:00 12/23/23
 * (also the driver and co-captain and CAD guy)
 * (basically i make robots go)
 * very modest and humble FRC god, 2016-2024
 * https://docs.google.com/document/d/1fm2T6eL4VdnDw-0bCEbM68ifCgFhKDjI2-HoGgEnTO8/
 */