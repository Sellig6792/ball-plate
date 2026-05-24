use std::env;
use dotenv::dotenv;

#[derive(Debug, Clone, Copy)]
pub enum Axe {
    X = 0,
    Y = 1,
}

pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub dt: f32,
}

struct PidState {
    erreur_precedente: f32,
    somme_integrale: f32,
}

impl PidState {
    fn new() -> Self {
        Self {
            erreur_precedente: 0.0,
            somme_integrale: 0.0,
        }
    }
}

pub struct BallAndPlatePid {
    config: PidConfig,
    etat_x: PidState,
    etat_y: PidState,

    // Coordonnées du centre automatique (en pixels)
    centre_x_pixel: f32,
    centre_y_pixel: f32,

    // Ratio pour convertir les pixels en centimètres
    // (Dépend de la distance de votre caméra)
    pixels_par_cm: f32,
}

impl BallAndPlatePid {
    /// Initialise le PID et effectue la calibration du centre
    /// - `balle_initiale_x` et `balle_initiale_y` : La position en pixels de la balle
    ///   lorsqu'elle est posée manuellement AU CENTRE de la plaque au démarrage.
    pub fn from_env(balle_initiale_x: f32, balle_initiale_y: f32) -> Self {
        dotenv().expect("Impossible de charger le fichier .env");

        let kp: f32 = env::var("PID_KP").expect("PID_KP manquant").parse().unwrap();
        let ki: f32 = env::var("PID_KI").expect("PID_KI manquant").parse().unwrap();
        let kd: f32 = env::var("PID_KD").expect("PID_KD manquant").parse().unwrap();
        let fps: f32 = env::var("frame_rate").expect("frame_rate manquant").parse().unwrap();

        // On récupère le facteur d'échelle (combien de pixels représentent 1 cm à l'écran)
        let pixels_par_cm: f32 = env::var("PIXELS_PAR_CM")
            .unwrap_or_else(|_| "10.0".to_string()) // Valeur par défaut si non définie
            .parse()
            .unwrap();

        println!("--- CALIBRATION DU CENTRE RÉUSSIE ---");
        println!("Centre enregistré à : X = {}px, Y = {}px", balle_initiale_x, balle_initiale_y);

        Self {
            config: PidConfig { kp, ki, kd, dt: 1.0 / fps },
            etat_x: PidState::new(),
            etat_y: PidState::new(),
            centre_x_pixel: balle_initiale_x,
            centre_y_pixel: balle_initiale_y,
            pixels_par_cm,
        }
    }

    /// Calcule l'inclinaison de la plaque
    /// - `axe`: X ou Y
    /// - `position_balle_pixel`: La coordonnée brute (0 à 640, ou 0 à 1920...) donnée par OpenCV
    pub fn calculer_inclinaison(&mut self, axe: Axe, position_balle_pixel: f32) -> f32 {
        let dt = self.config.dt;

        let (etat, centre_pixel) = match axe {
            Axe::X => (&mut self.etat_x, self.centre_x_pixel),
            Axe::Y => (&mut self.etat_y, self.centre_y_pixel),
        };

        // 1. Calcul de l'écart en pixels par rapport au centre calibré
        let ecart_pixel = centre_pixel - position_balle_pixel;

        // 2. Conversion de l'erreur en Centimètres (plus logique physiquement pour le PID)
        // Si la balle est à gauche du centre, l'erreur est positive, si elle est à droite, négative.
        let erreur_cm = ecart_pixel / self.pixels_par_cm;

        // Sécurité : On sature l'erreur à la taille max de votre plaque (40cm / 2 = 20cm du centre)
        let erreur_cm = erreur_cm.clamp(-20.0, 20.0);

        // --- CALCULS DU PID ---
        // Proportionnel
        let p = self.config.kp * erreur_cm;

        // Intégral
        if self.config.ki > 0.0 {
            etat.somme_integrale += erreur_cm * dt;
            // Limite de l'intégrale à 5cm.s pour éviter l'emballement
            etat.somme_integrale = etat.somme_integrale.clamp(-5.0, 5.0);
        }
        let i = self.config.ki * etat.somme_integrale;

        // Dérivé (Mesure la vitesse de la balle en cm/seconde)
        let d = if dt > 0.0 {
            self.config.kd * ((erreur_cm - etat.erreur_precedente) / dt)
        } else {
            0.0
        };

        // Sauvegarde de l'erreur
        etat.erreur_precedente = erreur_cm;

        // La commande finale modifie l'angle autour de la position "0.5" (la plaque à plat)
        // On divise par 100 ou un facteur de gain pour que la sortie reste douce.
        let commande_pid = (p + i + d) / 100.0;

        let inclinaison_plaque = 0.5 + commande_pid;

        // Sécurité stricte pour vos servomoteurs (ne penche pas à plus de 30% du max)
        inclinaison_plaque.clamp(0.2, 0.8)
    }
}